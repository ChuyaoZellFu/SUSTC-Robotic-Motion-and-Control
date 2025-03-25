;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FocusPoint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FocusPoint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FOCUSPOINT")
  (make-package "KORTEX_DRIVER::FOCUSPOINT"))

(in-package "ROS")
;;//! \htmlinclude FocusPoint.msg.html


(defclass kortex_driver::FocusPoint
  :super ros::object
  :slots (_x _y ))

(defmethod kortex_driver::FocusPoint
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint32 _x
    4
    ;; uint32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x
       (write-long _x s)
     ;; uint32 _y
       (write-long _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::FocusPoint :md5sum-) "64be90712af6ea79ae6f103da824ffcf")
(setf (get kortex_driver::FocusPoint :datatype-) "kortex_driver/FocusPoint")
(setf (get kortex_driver::FocusPoint :definition-)
      "
uint32 x
uint32 y
")



(provide :kortex_driver/FocusPoint "64be90712af6ea79ae6f103da824ffcf")


