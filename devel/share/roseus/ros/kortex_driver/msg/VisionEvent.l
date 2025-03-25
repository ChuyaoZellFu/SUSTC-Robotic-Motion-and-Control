;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::VisionEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'VisionEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::VISIONEVENT")
  (make-package "KORTEX_DRIVER::VISIONEVENT"))

(in-package "ROS")
;;//! \htmlinclude VisionEvent.msg.html


(intern "*UNSPECIFIED_VISION_EVENT*" (find-package "KORTEX_DRIVER::VISIONEVENT"))
(shadow '*UNSPECIFIED_VISION_EVENT* (find-package "KORTEX_DRIVER::VISIONEVENT"))
(defconstant kortex_driver::VisionEvent::*UNSPECIFIED_VISION_EVENT* 0)
(intern "*SENSOR_SETTINGS_CHANGED*" (find-package "KORTEX_DRIVER::VISIONEVENT"))
(shadow '*SENSOR_SETTINGS_CHANGED* (find-package "KORTEX_DRIVER::VISIONEVENT"))
(defconstant kortex_driver::VisionEvent::*SENSOR_SETTINGS_CHANGED* 1)
(intern "*OPTION_VALUE_CHANGED*" (find-package "KORTEX_DRIVER::VISIONEVENT"))
(shadow '*OPTION_VALUE_CHANGED* (find-package "KORTEX_DRIVER::VISIONEVENT"))
(defconstant kortex_driver::VisionEvent::*OPTION_VALUE_CHANGED* 2)

(defun kortex_driver::VisionEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::VisionEvent::*UNSPECIFIED_VISION_EVENT*)
        ((= const 1) 'kortex_driver::VisionEvent::*SENSOR_SETTINGS_CHANGED*)
        ((= const 2) 'kortex_driver::VisionEvent::*OPTION_VALUE_CHANGED*)
        (t nil)))

(defclass kortex_driver::VisionEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::VisionEvent
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

(setf (get kortex_driver::VisionEvent :md5sum-) "5d538f4fa32289d8ec7691c8a8d47f4b")
(setf (get kortex_driver::VisionEvent :datatype-) "kortex_driver/VisionEvent")
(setf (get kortex_driver::VisionEvent :definition-)
      "
uint32 UNSPECIFIED_VISION_EVENT = 0

uint32 SENSOR_SETTINGS_CHANGED = 1

uint32 OPTION_VALUE_CHANGED = 2

")



(provide :kortex_driver/VisionEvent "5d538f4fa32289d8ec7691c8a8d47f4b")


