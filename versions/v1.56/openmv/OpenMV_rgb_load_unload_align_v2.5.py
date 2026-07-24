# Boot in normal RGB mode with the camera aimed down at the claw. The red
# block must enter the calibrated capture window between the two fingers.
# After STM32 confirms pickup it sends MODE,LINE, then this script switches
# grayscale/binary continuous fitted-line processing from v1.9. At the
# unloading area STM32 sends MODE,UNLOAD to restore normal RGB. The carried
# red block is then aligned with the black circular drop point.
# STM32 -> OpenMV: MODE,RED, MODE,LINE or MODE,UNLOAD
# OpenMV -> STM32: MV,LOAD,... / MV,LINE,... / MV,UNLOAD,... /
#                    MV,UNLOAD_ENTRY,...
# v2.5: RGB red-block/black-circle relative alignment for unloading.

import sensor
import time
import gc
import math
from pyb import UART


VERSION = "SENSOR_RGB_UNLOAD2.5"

UART_ID = 3
UART_BAUD = 115200

MODE_RED = 0
MODE_LINE = 1
MODE_UNLOAD = 2
RED_THRESHOLD = (20, 85, 20, 90, 5, 75)
RED_MIN_PIXELS = 200
RED_MIN_AREA = 200
CAMERA_HORIZONTAL_FOV_DEG = 60
CAMERA_VERTICAL_FOV_DEG = 45

# Calibrate these four coordinates while viewing the RGB image in OpenMV IDE.
# The rectangle must cover the usable space between the two open claw fingers.
GRIP_ZONE_X_MIN = 110
GRIP_ZONE_X_MAX = 210
GRIP_ZONE_Y_MIN = 150
GRIP_ZONE_Y_MAX = 215

GRIP_NOT_READY = 0
GRIP_APPROACH = 1
GRIP_READY = 2
GRIP_TOO_CLOSE = 3

# Black unloading-circle geometry. LINE mode uses the inverted binary image
# only to detect entry. UNLOAD mode uses the normal RGB image and LAB black
# threshold so the carried red block and the black circle remain distinct.
UNLOAD_BLACK_THRESHOLD = (0, 38, -22, 22, -22, 22)
UNLOAD_RGB_MIN_PIXELS = 300
UNLOAD_RGB_MIN_AREA = 450
UNLOAD_RED_MIN_PIXELS = 120
UNLOAD_RED_MIN_AREA = 120
UNLOAD_ALIGN_Y_TOLERANCE = 24
UNLOAD_DETECT_ARM_DELAY_MS = 2000
UNLOAD_CONFIRM_FRAMES = 3
UNLOAD_MIN_WIDTH = 28
UNLOAD_MAX_WIDTH = 120
UNLOAD_MIN_HEIGHT = 28
UNLOAD_MAX_HEIGHT = 120
UNLOAD_ASPECT_MIN_PERCENT = 70
UNLOAD_ASPECT_MAX_PERCENT = 170
UNLOAD_FILL_MIN_PERCENT = 48
UNLOAD_FILL_MAX_PERCENT = 92
UNLOAD_EDGE_MARGIN = 5
UNLOAD_CONFIRM_CENTER_TOLERANCE = 18
UNLOAD_CONFIRM_SIZE_TOLERANCE = 20

# The unloading-area entrance is the wide horizontal white tape shown in the
# supplied binary image. It must span the centre and both sides for 3 frames.
UNLOAD_ENTRY_Y_MIN = 90
UNLOAD_ENTRY_Y_MAX = 155
UNLOAD_ENTRY_Y_STEP = 5
UNLOAD_ENTRY_MIN_WHITE = 230
UNLOAD_ENTRY_SIDE_WIDTH = 110
UNLOAD_ENTRY_MIN_SIDE_WHITE = 70
UNLOAD_ENTRY_CONFIRM_FRAMES = 3

UNLOAD_ZONE_X_MIN = 110
UNLOAD_ZONE_X_MAX = 210
UNLOAD_ZONE_Y_MIN = 145
UNLOAD_ZONE_Y_MAX = 210

UNLOAD_NOT_READY = 0
UNLOAD_APPROACH = 1
UNLOAD_READY = 2
UNLOAD_TOO_CLOSE = 3

IMG_WIDTH = 320
IMG_HEIGHT = 240
CAMERA_CENTER_X = 160
LINE_OFFSET_SIGN = 1

# Keep exactly the preprocessing used by F:\xx.py. Pixels in the original
# grayscale range 0..100 become white in the processed binary image.
THRESHOLD = (0, 100)
BINARY_LINE_MIN = 200

# Continuous horizontal sampling rows, bottom to top. These are sample lines,
# not rectangular ROIs. More rows improve fitting but reduce frame rate.
SAMPLE_BOTTOM_Y = 225
SAMPLE_TOP_Y = 85
SAMPLE_STEP_Y = 14
MIN_FIT_POINTS = 6
MIN_VERTICAL_SPAN = 45

# Valid width of one white line segment in the processed binary image.
MIN_LINE_RUN_WIDTH = 2
MAX_LINE_RUN_WIDTH = 60

# Once locked, only search close to the predicted fitted-line position.
# This spatial gate is the main protection against jumping to a nearby line.
LOCKED_SEARCH_HALF_WIDTH = 45
UNLOCKED_CHAIN_HALF_WIDTH = 70

# A substantially different fit must persist before replacing the current
# tracked line. Short glitches are sent as valid=0 instead of wrong steering.
TRACK_CONFIRM_FRAMES = 3
TRACK_LOCK_RELEASE_FRAMES = 8
TRACK_OFFSET_JUMP_MAX = 22
TRACK_HEADING_JUMP_MAX = 28
TRACK_PENDING_OFFSET_TOLERANCE = 12
TRACK_PENDING_HEADING_TOLERANCE = 16

NEAR_Y = 210
FAR_Y = 120
SEND_INTERVAL_MS = 40
DEBUG_PRINT_INTERVAL_MS = 500
FILTER_OLD_WEIGHT = 2
FILTER_NEW_WEIGHT = 3

# Q10 fixed-point coefficients are accurate enough and cheaper than floats.
FIT_SHIFT = 10
FIT_ONE = 1 << FIT_SHIFT


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def configure_camera(mode):
    sensor.reset()
    if mode == MODE_RED or mode == MODE_UNLOAD:
        sensor.set_pixformat(sensor.RGB565)
    else:
        sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(True)
    sensor.set_vflip(True)

    if mode == MODE_RED or mode == MODE_UNLOAD:
        # Let exposure and white balance settle in RGB mode, then lock them so
        # the LAB threshold does not drift during the final approach.
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
        sensor.skip_frames(time=800)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
    else:
        sensor.set_auto_gain(True)
        sensor.skip_frames(time=300)


def poll_mode_command(uart, current_mode, command_buffer):
    requested_mode = current_mode
    while uart.any():
        value = uart.readchar()
        if value is None or value < 0:
            break
        if value == 10 or value == 13:
            if command_buffer == "MODE,RED":
                requested_mode = MODE_RED
            elif command_buffer == "MODE,LINE":
                requested_mode = MODE_LINE
            elif command_buffer == "MODE,UNLOAD":
                requested_mode = MODE_UNLOAD
            command_buffer = ""
        elif len(command_buffer) < 31:
            command_buffer += chr(value)
        else:
            command_buffer = ""
    return requested_mode, command_buffer


def detect_red_load(img):
    blobs = img.find_blobs([RED_THRESHOLD],
                           pixels_threshold=RED_MIN_PIXELS,
                           area_threshold=RED_MIN_AREA,
                           merge=True)
    if not blobs:
        return 0, 0, 0, GRIP_NOT_READY, 0

    blob = max(blobs, key=lambda item: item.pixels)
    horizontal = ((blob.cx - CAMERA_CENTER_X) *
                  CAMERA_HORIZONTAL_FOV_DEG) // IMG_WIDTH
    vertical = ((IMG_HEIGHT // 2 - blob.cy) *
                CAMERA_VERTICAL_FOV_DEG) // IMG_HEIGHT
    if blob.cy < GRIP_ZONE_Y_MIN:
        grip_state = GRIP_APPROACH
    elif blob.cy > GRIP_ZONE_Y_MAX:
        grip_state = GRIP_TOO_CLOSE
    elif GRIP_ZONE_X_MIN <= blob.cx <= GRIP_ZONE_X_MAX:
        grip_state = GRIP_READY
    else:
        grip_state = GRIP_NOT_READY

    img.draw_rectangle(blob.rect, color=(255, 0, 0), thickness=2)
    img.draw_cross((blob.cx, blob.cy), color=(0, 255, 0), size=12)
    return horizontal, vertical, blob.pixels, grip_state, 1


def find_unload_red_reference(img):
    blobs = img.find_blobs([RED_THRESHOLD],
                           pixels_threshold=UNLOAD_RED_MIN_PIXELS,
                           area_threshold=UNLOAD_RED_MIN_AREA,
                           merge=True)
    if not blobs:
        return None
    return max(blobs, key=lambda item: item.pixels)


def find_unload_circle_rgb(img, red_blob):
    blobs = img.find_blobs([UNLOAD_BLACK_THRESHOLD],
                           pixels_threshold=UNLOAD_RGB_MIN_PIXELS,
                           area_threshold=UNLOAD_RGB_MIN_AREA,
                           merge=False)
    candidates = []
    for blob in blobs:
        x, y, width, height = blob.rect
        if (x <= UNLOAD_EDGE_MARGIN or y <= UNLOAD_EDGE_MARGIN or
                (x + width) >= (IMG_WIDTH - UNLOAD_EDGE_MARGIN) or
                (y + height) >= (IMG_HEIGHT - UNLOAD_EDGE_MARGIN)):
            continue
        if not (UNLOAD_MIN_WIDTH <= width <= UNLOAD_MAX_WIDTH and
                UNLOAD_MIN_HEIGHT <= height <= UNLOAD_MAX_HEIGHT):
            continue

        aspect_percent = width * 100 // max(1, height)
        fill_percent = blob.pixels * 100 // max(1, width * height)
        if not (UNLOAD_ASPECT_MIN_PERCENT <= aspect_percent <=
                UNLOAD_ASPECT_MAX_PERCENT):
            continue
        if not (UNLOAD_FILL_MIN_PERCENT <= fill_percent <=
                UNLOAD_FILL_MAX_PERCENT):
            continue

        # A compact isolated dark component is preferred. The red reference
        # is deliberately not removed from the image; its centre is the
        # physical cargo position that the ground circle must approach.
        candidates.append((blob.pixels, blob, fill_percent))

    if not candidates:
        return None

    _, blob, fill_percent = max(candidates, key=lambda item: item[0])
    if red_blob is None:
        return (blob, 0, 0, blob.pixels, UNLOAD_NOT_READY,
                fill_percent, 0)

    delta_x = blob.cx - red_blob.cx
    delta_y = blob.cy - red_blob.cy
    horizontal = (delta_x * CAMERA_HORIZONTAL_FOV_DEG) // IMG_WIDTH
    vertical = (-delta_y * CAMERA_VERTICAL_FOV_DEG) // IMG_HEIGHT

    # Horizontal alignment is handled from the signed horizontal field by
    # STM32. The position state describes only forward/backward alignment,
    # preventing a small X error from blocking the final approach.
    if delta_y < -UNLOAD_ALIGN_Y_TOLERANCE:
        position_state = UNLOAD_APPROACH
    elif delta_y > UNLOAD_ALIGN_Y_TOLERANCE:
        position_state = UNLOAD_TOO_CLOSE
    else:
        position_state = UNLOAD_READY

    return (blob, horizontal, vertical, blob.pixels, position_state,
            fill_percent, 1)


def find_unload_circle(img):
    blobs = img.find_blobs([(BINARY_LINE_MIN, 255)],
                           pixels_threshold=350,
                           area_threshold=500,
                           merge=False)
    candidates = []
    for blob in blobs:
        x, y, width, height = blob.rect
        if (x <= UNLOAD_EDGE_MARGIN or y <= UNLOAD_EDGE_MARGIN or
                (x + width) >= (IMG_WIDTH - UNLOAD_EDGE_MARGIN) or
                (y + height) >= (IMG_HEIGHT - UNLOAD_EDGE_MARGIN)):
            continue
        if not (UNLOAD_MIN_WIDTH <= width <= UNLOAD_MAX_WIDTH and
                UNLOAD_MIN_HEIGHT <= height <= UNLOAD_MAX_HEIGHT):
            continue

        aspect_percent = width * 100 // max(1, height)
        fill_percent = blob.pixels * 100 // max(1, width * height)
        if not (UNLOAD_ASPECT_MIN_PERCENT <= aspect_percent <=
                UNLOAD_ASPECT_MAX_PERCENT):
            continue
        if not (UNLOAD_FILL_MIN_PERCENT <= fill_percent <=
                UNLOAD_FILL_MAX_PERCENT):
            continue

        # Prefer the largest compact isolated component.
        candidates.append((blob.pixels, blob, fill_percent))

    if not candidates:
        return None

    _, blob, fill_percent = max(candidates, key=lambda item: item[0])
    horizontal = ((blob.cx - CAMERA_CENTER_X) *
                  CAMERA_HORIZONTAL_FOV_DEG) // IMG_WIDTH
    vertical = ((IMG_HEIGHT // 2 - blob.cy) *
                CAMERA_VERTICAL_FOV_DEG) // IMG_HEIGHT

    if blob.cy < UNLOAD_ZONE_Y_MIN:
        position_state = UNLOAD_APPROACH
    elif blob.cy > UNLOAD_ZONE_Y_MAX:
        position_state = UNLOAD_TOO_CLOSE
    elif UNLOAD_ZONE_X_MIN <= blob.cx <= UNLOAD_ZONE_X_MAX:
        position_state = UNLOAD_READY
    else:
        position_state = UNLOAD_NOT_READY

    return (blob, horizontal, vertical, blob.pixels,
            position_state, fill_percent)


def detect_unload_entry(img):
    for y in range(UNLOAD_ENTRY_Y_MIN, UNLOAD_ENTRY_Y_MAX + 1,
                   UNLOAD_ENTRY_Y_STEP):
        white_count = 0
        left_count = 0
        right_count = 0
        for x in range(IMG_WIDTH):
            if img.get_pixel((x, y)) >= BINARY_LINE_MIN:
                white_count += 1
                if x < UNLOAD_ENTRY_SIDE_WIDTH:
                    left_count += 1
                elif x >= (IMG_WIDTH - UNLOAD_ENTRY_SIDE_WIDTH):
                    right_count += 1

        if (white_count >= UNLOAD_ENTRY_MIN_WHITE and
                left_count >= UNLOAD_ENTRY_MIN_SIDE_WHITE and
                right_count >= UNLOAD_ENTRY_MIN_SIDE_WHITE):
            return True
    return False


def find_binary_line_runs(img, y, x_start, x_end):
    runs = []
    run_start = -1

    for x in range(x_start, x_end + 1):
        pixel = img.get_pixel((x, y))
        if pixel >= BINARY_LINE_MIN:
            if run_start < 0:
                run_start = x
        elif run_start >= 0:
            width = x - run_start
            if MIN_LINE_RUN_WIDTH <= width <= MAX_LINE_RUN_WIDTH:
                center = run_start + width // 2
                runs.append((center, width))
            run_start = -1

    if run_start >= 0:
        width = x_end - run_start + 1
        if MIN_LINE_RUN_WIDTH <= width <= MAX_LINE_RUN_WIDTH:
            center = run_start + width // 2
            runs.append((center, width))

    return runs


def choose_run(runs, preferred_x, locked):
    if not runs:
        return None

    if locked:
        # Position continuity has priority after lock.
        return min(runs, key=lambda run: abs(run[0] - preferred_x))

    # At startup prefer the line nearest the camera center. Width breaks ties
    # without allowing a distant, larger line to take over.
    return min(runs, key=lambda run:
               (abs(run[0] - preferred_x), -run[1]))


def fit_points_q10(points):
    point_count = len(points)
    if point_count < 2:
        return None

    sum_x = 0
    sum_y = 0
    sum_yy = 0
    sum_xy = 0
    min_y = IMG_HEIGHT
    max_y = 0

    for point_x, point_y in points:
        sum_x += point_x
        sum_y += point_y
        sum_yy += point_y * point_y
        sum_xy += point_x * point_y
        if point_y < min_y:
            min_y = point_y
        if point_y > max_y:
            max_y = point_y

    if (max_y - min_y) < MIN_VERTICAL_SPAN:
        return None

    denominator = point_count * sum_yy - sum_y * sum_y
    if denominator == 0:
        return None

    slope_q10 = ((point_count * sum_xy - sum_y * sum_x) << FIT_SHIFT) // denominator
    intercept_q10 = ((sum_x << FIT_SHIFT) - slope_q10 * sum_y) // point_count
    return slope_q10, intercept_q10


def fitted_x(slope_q10, intercept_q10, y):
    return clamp((slope_q10 * y + intercept_q10) // FIT_ONE,
                 0, IMG_WIDTH - 1)


def sample_continuous_line(img, locked, old_slope_q10, old_intercept_q10):
    points = []
    total_width = 0
    preferred_x = CAMERA_CENTER_X

    for y in range(SAMPLE_BOTTOM_Y, SAMPLE_TOP_Y - 1, -SAMPLE_STEP_Y):
        if locked:
            preferred_x = fitted_x(old_slope_q10, old_intercept_q10, y)
            half_width = LOCKED_SEARCH_HALF_WIDTH
            x_start = clamp(preferred_x - half_width, 0, IMG_WIDTH - 1)
            x_end = clamp(preferred_x + half_width, 0, IMG_WIDTH - 1)
        elif points:
            # Chain the current frame's samples upward from the last point.
            preferred_x = points[-1][0]
            half_width = UNLOCKED_CHAIN_HALF_WIDTH
            x_start = clamp(preferred_x - half_width, 0, IMG_WIDTH - 1)
            x_end = clamp(preferred_x + half_width, 0, IMG_WIDTH - 1)
        else:
            x_start = 0
            x_end = IMG_WIDTH - 1

        runs = find_binary_line_runs(img, y, x_start, x_end)
        run = choose_run(runs, preferred_x, locked or bool(points))
        if run is None:
            continue

        point_x, width = run
        points.append((point_x, y))
        total_width += width
        img.draw_cross((point_x, y), color=255, size=4)

    fit = fit_points_q10(points)
    if fit is None or len(points) < MIN_FIT_POINTS:
        return None, points, 0

    # Point coverage is the primary confidence. Consistent visible tape width
    # adds at most 20 points without changing the UART magnitude range.
    row_count = ((SAMPLE_BOTTOM_Y - SAMPLE_TOP_Y) // SAMPLE_STEP_Y) + 1
    coverage = len(points) * 80 // row_count
    width_score = min(20, total_width // max(1, len(points)))
    magnitude = clamp(coverage + width_score, 0, 100)
    return fit, points, magnitude


def main():
    uart = UART(UART_ID, UART_BAUD, timeout_char=20)
    processing_mode = MODE_RED
    command_buffer = ""
    configure_camera(processing_mode)
    clock = time.clock()

    last_send_ms = time.ticks_ms()
    last_debug_ms = last_send_ms
    filtered_offset = 0
    filtered_heading = 0
    filter_valid = False

    track_locked = False
    locked_offset = 0
    locked_heading = 0
    locked_slope_q10 = 0
    locked_intercept_q10 = CAMERA_CENTER_X << FIT_SHIFT
    pending_offset = 0
    pending_heading = 0
    pending_count = 0
    lock_miss_frames = 0
    frame_count = 0
    line_mode_start_ms = 0
    unload_confirm_count = 0
    unload_last_cx = 0
    unload_last_cy = 0
    unload_last_width = 0
    unload_horizontal = 0
    unload_vertical = 0
    unload_area = 0
    unload_state = UNLOAD_NOT_READY
    unload_valid = 0
    unload_fill = 0
    unload_entry_confirm_count = 0
    unload_entry_latched = False

    while True:
        clock.tick()
        requested_mode, command_buffer = poll_mode_command(
            uart, processing_mode, command_buffer)
        if requested_mode != processing_mode:
            processing_mode = requested_mode
            configure_camera(processing_mode)
            # A new line session must not inherit a fit from the previous one.
            track_locked = False
            filter_valid = False
            pending_count = 0
            lock_miss_frames = 0
            unload_confirm_count = 0
            unload_valid = 0
            unload_entry_confirm_count = 0
            unload_entry_latched = False
            if processing_mode == MODE_LINE:
                line_mode_start_ms = time.ticks_ms()
            if processing_mode == MODE_LINE:
                mode_name = "LINE"
            elif processing_mode == MODE_UNLOAD:
                mode_name = "UNLOAD"
            else:
                mode_name = "RED"
            print("MODE=%s" % mode_name)

        if processing_mode == MODE_RED:
            img = sensor.snapshot()
            horizontal, vertical, area, grip_state, valid = detect_red_load(img)
            zone_color = (0, 255, 0) if grip_state == GRIP_READY else (255, 255, 0)
            img.draw_rectangle((GRIP_ZONE_X_MIN, GRIP_ZONE_Y_MIN,
                                GRIP_ZONE_X_MAX - GRIP_ZONE_X_MIN,
                                GRIP_ZONE_Y_MAX - GRIP_ZONE_Y_MIN),
                               color=zone_color, thickness=2)
            img.draw_line((CAMERA_CENTER_X, 0, CAMERA_CENTER_X,
                           IMG_HEIGHT - 1), color=(0, 255, 0))

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                last_send_ms = now_ms
                uart.write("MV,LOAD,%d,%d,%d,%d,%d\r\n" %
                           (horizontal, vertical, area, grip_state, valid))

            if time.ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_INTERVAL_MS:
                last_debug_ms = now_ms
                print("RED_%s FPS=%.1f H=%d V=%d AREA=%d GRIP=%d VALID=%d" %
                      (VERSION, clock.fps(), horizontal, vertical, area,
                       grip_state, valid))
            continue

        if processing_mode == MODE_UNLOAD:
            img = sensor.snapshot()
            red_blob = find_unload_red_reference(img)
            unload_candidate = find_unload_circle_rgb(img, red_blob)
            unload_blob = None
            unload_valid = 0
            unload_fill = 0

            if unload_candidate is not None:
                (unload_blob, unload_horizontal, unload_vertical, unload_area,
                 unload_state, unload_fill,
                 unload_reference_valid) = unload_candidate
                _, _, unload_width, _ = unload_blob.rect
                if (unload_reference_valid and unload_confirm_count > 0 and
                        abs(unload_blob.cx - unload_last_cx) <=
                        UNLOAD_CONFIRM_CENTER_TOLERANCE and
                        abs(unload_blob.cy - unload_last_cy) <=
                        UNLOAD_CONFIRM_CENTER_TOLERANCE and
                        abs(unload_width - unload_last_width) <=
                        UNLOAD_CONFIRM_SIZE_TOLERANCE):
                    unload_confirm_count += 1
                elif unload_reference_valid:
                    unload_confirm_count = 1
                else:
                    unload_confirm_count = 0
                unload_last_cx = unload_blob.cx
                unload_last_cy = unload_blob.cy
                unload_last_width = unload_width
                unload_valid = (
                    1 if unload_confirm_count >= UNLOAD_CONFIRM_FRAMES else 0)
            else:
                unload_confirm_count = 0
                unload_horizontal = 0
                unload_vertical = 0
                unload_area = 0
                unload_state = UNLOAD_NOT_READY

            if red_blob is not None:
                img.draw_rectangle(red_blob.rect, color=(255, 0, 0),
                                   thickness=2)
                img.draw_cross((red_blob.cx, red_blob.cy),
                               color=(0, 255, 0), size=12)
            if unload_blob is not None:
                circle_color = ((0, 255, 0) if unload_valid else
                                (255, 255, 0))
                img.draw_rectangle(unload_blob.rect, color=circle_color,
                                   thickness=3)
                img.draw_cross((unload_blob.cx, unload_blob.cy),
                               color=circle_color, size=12)
                if red_blob is not None:
                    img.draw_line((red_blob.cx, red_blob.cy,
                                   unload_blob.cx, unload_blob.cy),
                                  color=(0, 255, 255), thickness=2)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                last_send_ms = now_ms
                uart.write("MV,UNLOAD,%d,%d,%d,%d,%d\r\n" %
                           (unload_horizontal, unload_vertical, unload_area,
                            unload_state, unload_valid))

            if time.ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_INTERVAL_MS:
                last_debug_ms = now_ms
                print("UNLOAD_RGB_%s FPS=%.1f H=%d V=%d AREA=%d "
                      "STATE=%d VALID=%d RED=%d CONF=%d FILL=%d" %
                      (VERSION, clock.fps(), unload_horizontal,
                       unload_vertical, unload_area, unload_state,
                       unload_valid, 1 if red_blob is not None else 0,
                       unload_confirm_count, unload_fill))
            continue

        source = sensor.snapshot()
        # Recognition runs on exactly the processed image produced by xx.py:
        # original black pixels in THRESHOLD become white line pixels.
        img = source.binary([THRESHOLD])

        unload_candidate = None
        unload_blob = None
        unload_entry_candidate = False
        now_ms = time.ticks_ms()
        if (line_mode_start_ms != 0 and
                time.ticks_diff(now_ms, line_mode_start_ms) >=
                UNLOAD_DETECT_ARM_DELAY_MS):
            unload_candidate = find_unload_circle(img)
            unload_entry_candidate = detect_unload_entry(img)

        if not unload_entry_latched:
            if unload_entry_candidate:
                unload_entry_confirm_count += 1
                if unload_entry_confirm_count >= UNLOAD_ENTRY_CONFIRM_FRAMES:
                    unload_entry_latched = True
            else:
                unload_entry_confirm_count = 0

        unload_valid = 0
        if unload_candidate is not None:
            (unload_blob, unload_horizontal, unload_vertical, unload_area,
             unload_state, unload_fill) = unload_candidate
            _, _, unload_width, _ = unload_blob.rect
            if (unload_confirm_count > 0 and
                    abs(unload_blob.cx - unload_last_cx) <=
                    UNLOAD_CONFIRM_CENTER_TOLERANCE and
                    abs(unload_blob.cy - unload_last_cy) <=
                    UNLOAD_CONFIRM_CENTER_TOLERANCE and
                    abs(unload_width - unload_last_width) <=
                    UNLOAD_CONFIRM_SIZE_TOLERANCE):
                unload_confirm_count += 1
            else:
                unload_confirm_count = 1
            unload_last_cx = unload_blob.cx
            unload_last_cy = unload_blob.cy
            unload_last_width = unload_width
            unload_valid = 1 if unload_confirm_count >= UNLOAD_CONFIRM_FRAMES else 0
        else:
            unload_confirm_count = 0
            unload_horizontal = 0
            unload_vertical = 0
            unload_area = 0
            unload_state = UNLOAD_NOT_READY
            unload_fill = 0

        fit, track_points, magnitude = sample_continuous_line(
            img, track_locked, locked_slope_q10, locked_intercept_q10)

        valid = 0
        raw_offset = 0
        raw_heading = 0
        near_x = CAMERA_CENTER_X
        far_x = CAMERA_CENTER_X
        candidate_present = fit is not None

        if fit is not None:
            slope_q10, intercept_q10 = fit
            near_x = fitted_x(slope_q10, intercept_q10, NEAR_Y)
            far_x = fitted_x(slope_q10, intercept_q10, FAR_Y)
            raw_offset = (near_x - CAMERA_CENTER_X) * LINE_OFFSET_SIGN
            raw_heading = (far_x - near_x) * LINE_OFFSET_SIGN

            candidate_accepted = False
            if (track_locked and
                    abs(raw_offset - locked_offset) <= TRACK_OFFSET_JUMP_MAX and
                    abs(raw_heading - locked_heading) <= TRACK_HEADING_JUMP_MAX):
                candidate_accepted = True
                pending_count = 0
            else:
                if (pending_count > 0 and
                        abs(raw_offset - pending_offset) <=
                        TRACK_PENDING_OFFSET_TOLERANCE and
                        abs(raw_heading - pending_heading) <=
                        TRACK_PENDING_HEADING_TOLERANCE):
                    pending_count += 1
                    pending_offset = (pending_offset + raw_offset) // 2
                    pending_heading = (pending_heading + raw_heading) // 2
                else:
                    pending_offset = raw_offset
                    pending_heading = raw_heading
                    pending_count = 1

                if pending_count >= TRACK_CONFIRM_FRAMES:
                    candidate_accepted = True
                    pending_count = 0

            if candidate_accepted:
                track_locked = True
                locked_offset = raw_offset
                locked_heading = raw_heading
                locked_slope_q10 = slope_q10
                locked_intercept_q10 = intercept_q10
                lock_miss_frames = 0

                if filter_valid:
                    filtered_offset = (
                        filtered_offset * FILTER_OLD_WEIGHT +
                        raw_offset * FILTER_NEW_WEIGHT
                    ) // (FILTER_OLD_WEIGHT + FILTER_NEW_WEIGHT)
                    filtered_heading = (
                        filtered_heading * FILTER_OLD_WEIGHT +
                        raw_heading * FILTER_NEW_WEIGHT
                    ) // (FILTER_OLD_WEIGHT + FILTER_NEW_WEIGHT)
                else:
                    filtered_offset = raw_offset
                    filtered_heading = raw_heading
                    filter_valid = True

                valid = 1
                img.draw_line((near_x, NEAR_Y, far_x, FAR_Y),
                              color=255, thickness=2)
                img.draw_cross((near_x, NEAR_Y), color=255, size=8)
                img.draw_cross((far_x, FAR_Y), color=255, size=8)

        if not valid:
            filter_valid = False
            filtered_offset = 0
            filtered_heading = 0
            lock_miss_frames += 1
            if not candidate_present:
                pending_count = 0
            if lock_miss_frames >= TRACK_LOCK_RELEASE_FRAMES:
                track_locked = False

        # Draw circle diagnostics only after line sampling so overlays cannot
        # become false line pixels in the same frame.
        if unload_blob is not None:
            unload_blob_color = 255 if unload_valid else 160
            img.draw_rectangle(unload_blob.rect, color=unload_blob_color,
                               thickness=3)
            img.draw_cross((unload_blob.cx, unload_blob.cy),
                           color=unload_blob_color, size=12)
        img.draw_rectangle((UNLOAD_ZONE_X_MIN, UNLOAD_ZONE_Y_MIN,
                            UNLOAD_ZONE_X_MAX - UNLOAD_ZONE_X_MIN,
                            UNLOAD_ZONE_Y_MAX - UNLOAD_ZONE_Y_MIN),
                           color=160, thickness=1)

        img.draw_line((CAMERA_CENTER_X, 0,
                       CAMERA_CENTER_X, IMG_HEIGHT - 1), color=160)
        img.draw_line((0, NEAR_Y, IMG_WIDTH - 1, NEAR_Y), color=160)
        img.draw_line((0, FAR_Y, IMG_WIDTH - 1, FAR_Y), color=160)

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
            last_send_ms = now_ms
            uart.write("MV,LINE,%d,%d,%d,%d\r\n" %
                       (filtered_offset, filtered_heading,
                        magnitude, valid))
            uart.write("MV,UNLOAD,%d,%d,%d,%d,%d\r\n" %
                       (unload_horizontal, unload_vertical, unload_area,
                        unload_state, unload_valid))
            uart.write("MV,UNLOAD_ENTRY,%d\r\n" %
                       (1 if unload_entry_latched else 0))

        if time.ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_INTERVAL_MS:
            last_debug_ms = now_ms
            angle = int(math.degrees(math.atan(
                filtered_offset / (IMG_HEIGHT / 2)))) if valid else 0
            print("LINE_%s FPS=%.1f NEAR=%d FAR=%d OFF=%d HEAD=%d "
                  "ANGLE=%d POINTS=%d VALID=%d MAG=%d LOCK=%d PENDING=%d "
                  "CIRCLE=%d STATE=%d AREA=%d FILL=%d CONF=%d ENTRY=%d/%d" %
                  (VERSION, clock.fps(), near_x, far_x, filtered_offset,
                   filtered_heading, angle, len(track_points), valid,
                   magnitude, 1 if track_locked else 0, pending_count,
                   unload_valid, unload_state, unload_area, unload_fill,
                   unload_confirm_count, 1 if unload_entry_latched else 0,
                   unload_entry_confirm_count))

        frame_count += 1
        if frame_count % 100 == 0:
            gc.collect()


main()
