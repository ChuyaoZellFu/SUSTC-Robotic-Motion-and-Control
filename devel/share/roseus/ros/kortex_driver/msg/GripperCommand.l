;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERCOMMAND")
  (make-package "KORTEX_DRIVER::GRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GripperCommand.msg.html


(defclass kortex_driver::GripperCommand
  :super ros::object
  :slots (_mode _gripper _duration ))

(defmethod kortex_driver::GripperCommand
  (:init
   (&key
    ((:mode __mode) 0)
    ((:gripper __gripper) (instance kortex_driver::Gripper :init))
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _gripper __gripper)
   (setq _duration (round __duration))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:gripper
   (&rest __gripper)
   (if (keywordp (car __gripper))
       (send* _gripper __gripper)
     (progn
       (if __gripper (setq _gripper (car __gripper)))
       _gripper)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _mode
    4
    ;; kortex_driver/Gripper _gripper
    (send _gripper :serialization-length)
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _mode
       (write-long _mode s)
     ;; kortex_driver/Gripper _gripper
       (send _gripper :serialize s)
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Gripper _gripper
     (send _gripper :deserialize buf ptr-) (incf ptr- (send _gripper :serialization-length))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::GripperCommand :md5sum-) "815b73064034a883f36452c4cb60c73f")
(setf (get kortex_driver::GripperCommand :datatype-) "kortex_driver/GripperCommand")
(setf (get kortex_driver::GripperCommand :definition-)
      "
uint32 mode
Gripper gripper
uint32 duration
================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
float32 value
")



(provide :kortex_driver/GripperCommand "815b73064034a883f36452c4cb60c73f")


