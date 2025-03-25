;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Option)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Option (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPTION")
  (make-package "KORTEX_DRIVER::OPTION"))

(in-package "ROS")
;;//! \htmlinclude Option.msg.html


(intern "*OPTION_UNSPECIFIED*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_UNSPECIFIED* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_UNSPECIFIED* 0)
(intern "*OPTION_BACKLIGHT_COMPENSATION*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_BACKLIGHT_COMPENSATION* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_BACKLIGHT_COMPENSATION* 1)
(intern "*OPTION_BRIGHTNESS*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_BRIGHTNESS* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_BRIGHTNESS* 2)
(intern "*OPTION_CONTRAST*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_CONTRAST* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_CONTRAST* 3)
(intern "*OPTION_EXPOSURE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_EXPOSURE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_EXPOSURE* 4)
(intern "*OPTION_GAIN*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_GAIN* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_GAIN* 5)
(intern "*OPTION_GAMMA*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_GAMMA* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_GAMMA* 6)
(intern "*OPTION_HUE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_HUE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_HUE* 7)
(intern "*OPTION_SATURATION*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_SATURATION* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_SATURATION* 8)
(intern "*OPTION_SHARPNESS*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_SHARPNESS* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_SHARPNESS* 9)
(intern "*OPTION_WHITE_BALANCE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_WHITE_BALANCE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_WHITE_BALANCE* 10)
(intern "*OPTION_ENABLE_AUTO_EXPOSURE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ENABLE_AUTO_EXPOSURE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ENABLE_AUTO_EXPOSURE* 11)
(intern "*OPTION_ENABLE_AUTO_WHITE_BALANCE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ENABLE_AUTO_WHITE_BALANCE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ENABLE_AUTO_WHITE_BALANCE* 12)
(intern "*OPTION_VISUAL_PRESET*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_VISUAL_PRESET* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_VISUAL_PRESET* 13)
(intern "*OPTION_LASER_POWER*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_LASER_POWER* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_LASER_POWER* 14)
(intern "*OPTION_ACCURACY*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ACCURACY* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ACCURACY* 15)
(intern "*OPTION_MOTION_RANGE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_MOTION_RANGE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_MOTION_RANGE* 16)
(intern "*OPTION_FILTER_OPTION*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_FILTER_OPTION* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_FILTER_OPTION* 17)
(intern "*OPTION_CONFIDENCE_THRESHOLD*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_CONFIDENCE_THRESHOLD* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_CONFIDENCE_THRESHOLD* 18)
(intern "*OPTION_EMITTER_ENABLED*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_EMITTER_ENABLED* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_EMITTER_ENABLED* 19)
(intern "*OPTION_FRAMES_QUEUE_SIZE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_FRAMES_QUEUE_SIZE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_FRAMES_QUEUE_SIZE* 20)
(intern "*OPTION_TOTAL_FRAME_DROPS*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_TOTAL_FRAME_DROPS* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_TOTAL_FRAME_DROPS* 21)
(intern "*OPTION_AUTO_EXPOSURE_MODE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_AUTO_EXPOSURE_MODE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_AUTO_EXPOSURE_MODE* 22)
(intern "*OPTION_POWER_LINE_FREQUENCY*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_POWER_LINE_FREQUENCY* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_POWER_LINE_FREQUENCY* 23)
(intern "*OPTION_ASIC_TEMPERATURE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ASIC_TEMPERATURE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ASIC_TEMPERATURE* 24)
(intern "*OPTION_ERROR_POLLING_ENABLED*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ERROR_POLLING_ENABLED* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ERROR_POLLING_ENABLED* 25)
(intern "*OPTION_PROJECTOR_TEMPERATURE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_PROJECTOR_TEMPERATURE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_PROJECTOR_TEMPERATURE* 26)
(intern "*OPTION_OUTPUT_TRIGGER_ENABLED*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_OUTPUT_TRIGGER_ENABLED* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_OUTPUT_TRIGGER_ENABLED* 27)
(intern "*OPTION_MOTION_MODULE_TEMPERATURE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_MOTION_MODULE_TEMPERATURE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_MOTION_MODULE_TEMPERATURE* 28)
(intern "*OPTION_DEPTH_UNITS*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_DEPTH_UNITS* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_DEPTH_UNITS* 29)
(intern "*OPTION_ENABLE_MOTION_CORRECTION*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_ENABLE_MOTION_CORRECTION* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_ENABLE_MOTION_CORRECTION* 30)
(intern "*OPTION_AUTO_EXPOSURE_PRIORITY*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_AUTO_EXPOSURE_PRIORITY* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_AUTO_EXPOSURE_PRIORITY* 31)
(intern "*OPTION_COLOR_SCHEME*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_COLOR_SCHEME* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_COLOR_SCHEME* 32)
(intern "*OPTION_HISTOGRAM_EQUALIZATION_ENABLED*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_HISTOGRAM_EQUALIZATION_ENABLED* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_HISTOGRAM_EQUALIZATION_ENABLED* 33)
(intern "*OPTION_MIN_DISTANCE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_MIN_DISTANCE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_MIN_DISTANCE* 34)
(intern "*OPTION_MAX_DISTANCE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_MAX_DISTANCE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_MAX_DISTANCE* 35)
(intern "*OPTION_TEXTURE_SOURCE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_TEXTURE_SOURCE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_TEXTURE_SOURCE* 36)
(intern "*OPTION_FILTER_MAGNITUDE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_FILTER_MAGNITUDE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_FILTER_MAGNITUDE* 37)
(intern "*OPTION_FILTER_SMOOTH_ALPHA*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_FILTER_SMOOTH_ALPHA* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_FILTER_SMOOTH_ALPHA* 38)
(intern "*OPTION_FILTER_SMOOTH_DELTA*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_FILTER_SMOOTH_DELTA* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_FILTER_SMOOTH_DELTA* 39)
(intern "*OPTION_HOLES_FILL*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_HOLES_FILL* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_HOLES_FILL* 40)
(intern "*OPTION_STEREO_BASELINE*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_STEREO_BASELINE* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_STEREO_BASELINE* 41)
(intern "*OPTION_AUTO_EXPOSURE_CONVERGE_STEP*" (find-package "KORTEX_DRIVER::OPTION"))
(shadow '*OPTION_AUTO_EXPOSURE_CONVERGE_STEP* (find-package "KORTEX_DRIVER::OPTION"))
(defconstant kortex_driver::Option::*OPTION_AUTO_EXPOSURE_CONVERGE_STEP* 42)

(defun kortex_driver::Option-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Option::*OPTION_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::Option::*OPTION_BACKLIGHT_COMPENSATION*)
        ((= const 2) 'kortex_driver::Option::*OPTION_BRIGHTNESS*)
        ((= const 3) 'kortex_driver::Option::*OPTION_CONTRAST*)
        ((= const 4) 'kortex_driver::Option::*OPTION_EXPOSURE*)
        ((= const 5) 'kortex_driver::Option::*OPTION_GAIN*)
        ((= const 6) 'kortex_driver::Option::*OPTION_GAMMA*)
        ((= const 7) 'kortex_driver::Option::*OPTION_HUE*)
        ((= const 8) 'kortex_driver::Option::*OPTION_SATURATION*)
        ((= const 9) 'kortex_driver::Option::*OPTION_SHARPNESS*)
        ((= const 10) 'kortex_driver::Option::*OPTION_WHITE_BALANCE*)
        ((= const 11) 'kortex_driver::Option::*OPTION_ENABLE_AUTO_EXPOSURE*)
        ((= const 12) 'kortex_driver::Option::*OPTION_ENABLE_AUTO_WHITE_BALANCE*)
        ((= const 13) 'kortex_driver::Option::*OPTION_VISUAL_PRESET*)
        ((= const 14) 'kortex_driver::Option::*OPTION_LASER_POWER*)
        ((= const 15) 'kortex_driver::Option::*OPTION_ACCURACY*)
        ((= const 16) 'kortex_driver::Option::*OPTION_MOTION_RANGE*)
        ((= const 17) 'kortex_driver::Option::*OPTION_FILTER_OPTION*)
        ((= const 18) 'kortex_driver::Option::*OPTION_CONFIDENCE_THRESHOLD*)
        ((= const 19) 'kortex_driver::Option::*OPTION_EMITTER_ENABLED*)
        ((= const 20) 'kortex_driver::Option::*OPTION_FRAMES_QUEUE_SIZE*)
        ((= const 21) 'kortex_driver::Option::*OPTION_TOTAL_FRAME_DROPS*)
        ((= const 22) 'kortex_driver::Option::*OPTION_AUTO_EXPOSURE_MODE*)
        ((= const 23) 'kortex_driver::Option::*OPTION_POWER_LINE_FREQUENCY*)
        ((= const 24) 'kortex_driver::Option::*OPTION_ASIC_TEMPERATURE*)
        ((= const 25) 'kortex_driver::Option::*OPTION_ERROR_POLLING_ENABLED*)
        ((= const 26) 'kortex_driver::Option::*OPTION_PROJECTOR_TEMPERATURE*)
        ((= const 27) 'kortex_driver::Option::*OPTION_OUTPUT_TRIGGER_ENABLED*)
        ((= const 28) 'kortex_driver::Option::*OPTION_MOTION_MODULE_TEMPERATURE*)
        ((= const 29) 'kortex_driver::Option::*OPTION_DEPTH_UNITS*)
        ((= const 30) 'kortex_driver::Option::*OPTION_ENABLE_MOTION_CORRECTION*)
        ((= const 31) 'kortex_driver::Option::*OPTION_AUTO_EXPOSURE_PRIORITY*)
        ((= const 32) 'kortex_driver::Option::*OPTION_COLOR_SCHEME*)
        ((= const 33) 'kortex_driver::Option::*OPTION_HISTOGRAM_EQUALIZATION_ENABLED*)
        ((= const 34) 'kortex_driver::Option::*OPTION_MIN_DISTANCE*)
        ((= const 35) 'kortex_driver::Option::*OPTION_MAX_DISTANCE*)
        ((= const 36) 'kortex_driver::Option::*OPTION_TEXTURE_SOURCE*)
        ((= const 37) 'kortex_driver::Option::*OPTION_FILTER_MAGNITUDE*)
        ((= const 38) 'kortex_driver::Option::*OPTION_FILTER_SMOOTH_ALPHA*)
        ((= const 39) 'kortex_driver::Option::*OPTION_FILTER_SMOOTH_DELTA*)
        ((= const 40) 'kortex_driver::Option::*OPTION_HOLES_FILL*)
        ((= const 41) 'kortex_driver::Option::*OPTION_STEREO_BASELINE*)
        ((= const 42) 'kortex_driver::Option::*OPTION_AUTO_EXPOSURE_CONVERGE_STEP*)
        (t nil)))

(defclass kortex_driver::Option
  :super ros::object
  :slots ())

(defmethod kortex_driver::Option
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::Option :md5sum-) "0f091628538d1f237d2c9bbe103cea73")
(setf (get kortex_driver::Option :datatype-) "kortex_driver/Option")
(setf (get kortex_driver::Option :definition-)
      "
uint32 OPTION_UNSPECIFIED = 0

uint32 OPTION_BACKLIGHT_COMPENSATION = 1

uint32 OPTION_BRIGHTNESS = 2

uint32 OPTION_CONTRAST = 3

uint32 OPTION_EXPOSURE = 4

uint32 OPTION_GAIN = 5

uint32 OPTION_GAMMA = 6

uint32 OPTION_HUE = 7

uint32 OPTION_SATURATION = 8

uint32 OPTION_SHARPNESS = 9

uint32 OPTION_WHITE_BALANCE = 10

uint32 OPTION_ENABLE_AUTO_EXPOSURE = 11

uint32 OPTION_ENABLE_AUTO_WHITE_BALANCE = 12

uint32 OPTION_VISUAL_PRESET = 13

uint32 OPTION_LASER_POWER = 14

uint32 OPTION_ACCURACY = 15

uint32 OPTION_MOTION_RANGE = 16

uint32 OPTION_FILTER_OPTION = 17

uint32 OPTION_CONFIDENCE_THRESHOLD = 18

uint32 OPTION_EMITTER_ENABLED = 19

uint32 OPTION_FRAMES_QUEUE_SIZE = 20

uint32 OPTION_TOTAL_FRAME_DROPS = 21

uint32 OPTION_AUTO_EXPOSURE_MODE = 22

uint32 OPTION_POWER_LINE_FREQUENCY = 23

uint32 OPTION_ASIC_TEMPERATURE = 24

uint32 OPTION_ERROR_POLLING_ENABLED = 25

uint32 OPTION_PROJECTOR_TEMPERATURE = 26

uint32 OPTION_OUTPUT_TRIGGER_ENABLED = 27

uint32 OPTION_MOTION_MODULE_TEMPERATURE = 28

uint32 OPTION_DEPTH_UNITS = 29

uint32 OPTION_ENABLE_MOTION_CORRECTION = 30

uint32 OPTION_AUTO_EXPOSURE_PRIORITY = 31

uint32 OPTION_COLOR_SCHEME = 32

uint32 OPTION_HISTOGRAM_EQUALIZATION_ENABLED = 33

uint32 OPTION_MIN_DISTANCE = 34

uint32 OPTION_MAX_DISTANCE = 35

uint32 OPTION_TEXTURE_SOURCE = 36

uint32 OPTION_FILTER_MAGNITUDE = 37

uint32 OPTION_FILTER_SMOOTH_ALPHA = 38

uint32 OPTION_FILTER_SMOOTH_DELTA = 39

uint32 OPTION_HOLES_FILL = 40

uint32 OPTION_STEREO_BASELINE = 41

uint32 OPTION_AUTO_EXPOSURE_CONVERGE_STEP = 42

")



(provide :kortex_driver/Option "0f091628538d1f237d2c9bbe103cea73")


