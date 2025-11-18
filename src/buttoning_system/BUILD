# Bazel BUILD file for MediaPipe hand tracking test
# Place this in: c:/Users/HaveS/source/Ros2ws/src/buttoning/src/buttoning_system/BUILD

load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "mediapipe_hand_test",
    srcs = ["src/mediapipe_hand_test.cpp"],
    deps = [
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:image_frame_opencv",
        "@mediapipe//mediapipe/framework/formats:landmark_cc_proto",
        "@mediapipe//mediapipe/framework/port:file_helpers",
        "@mediapipe//mediapipe/framework/port:parse_text_proto",
        "@mediapipe//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/status",
    ],
    data = [
        "@mediapipe//mediapipe/graphs/hand_tracking:hand_tracking_desktop_live.pbtxt",
        "@mediapipe//mediapipe/modules/hand_landmark:hand_landmark_tracking_cpu.pbtxt",
    ],
)
