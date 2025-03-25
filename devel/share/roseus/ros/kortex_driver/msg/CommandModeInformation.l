;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CommandModeInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CommandModeInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMMANDMODEINFORMATION")
  (make-package "KORTEX_DRIVER::COMMANDMODEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude CommandModeInformation.msg.html


(defclass kortex_driver::CommandModeInformation
  :super ros::object
  :slots (_command_mode ))

(defmethod kortex_driver::CommandModeInformation
  (:init
   (&key
    ((:command_mode __command_mode) 0)
    )
   (send-super :init)
   (setq _command_mode (round __command_mode))
   self)
  (:command_mode
   (&optional __command_mode)
   (if __command_mode (setq _command_mode __command_mode)) _command_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _command_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _command_mode
       (write-long _command_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _command_mode
     (setq _command_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CommandModeInformation :md5sum-) "7d0d1fb206a87d77cdd9700eb23ea5cd")
(setf (get kortex_driver::CommandModeInformation :datatype-) "kortex_driver/CommandModeInformation")
(setf (get kortex_driver::CommandModeInformation :definition-)
      "
uint32 command_mode
")



(provide :kortex_driver/CommandModeInformation "7d0d1fb206a87d77cdd9700eb23ea5cd")


