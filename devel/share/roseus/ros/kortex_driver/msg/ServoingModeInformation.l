;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ServoingModeInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ServoingModeInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SERVOINGMODEINFORMATION")
  (make-package "KORTEX_DRIVER::SERVOINGMODEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ServoingModeInformation.msg.html


(defclass kortex_driver::ServoingModeInformation
  :super ros::object
  :slots (_servoing_mode ))

(defmethod kortex_driver::ServoingModeInformation
  (:init
   (&key
    ((:servoing_mode __servoing_mode) 0)
    )
   (send-super :init)
   (setq _servoing_mode (round __servoing_mode))
   self)
  (:servoing_mode
   (&optional __servoing_mode)
   (if __servoing_mode (setq _servoing_mode __servoing_mode)) _servoing_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _servoing_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _servoing_mode
       (write-long _servoing_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _servoing_mode
     (setq _servoing_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ServoingModeInformation :md5sum-) "fa12f11eee2104f3edd1a6487bc94db9")
(setf (get kortex_driver::ServoingModeInformation :datatype-) "kortex_driver/ServoingModeInformation")
(setf (get kortex_driver::ServoingModeInformation :definition-)
      "
uint32 servoing_mode
")



(provide :kortex_driver/ServoingModeInformation "fa12f11eee2104f3edd1a6487bc94db9")


