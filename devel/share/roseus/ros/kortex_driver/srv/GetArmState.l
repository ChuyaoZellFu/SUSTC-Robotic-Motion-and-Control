;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetArmState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetArmState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETARMSTATE")
  (make-package "KORTEX_DRIVER::GETARMSTATE"))
(unless (find-package "KORTEX_DRIVER::GETARMSTATEREQUEST")
  (make-package "KORTEX_DRIVER::GETARMSTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETARMSTATERESPONSE")
  (make-package "KORTEX_DRIVER::GETARMSTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetArmStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetArmStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetArmStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetArmStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ArmStateInformation :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ArmStateInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ArmStateInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ArmStateInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetArmState
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetArmState :md5sum-) "09192f2d3f6462e1466a4468f2c2390a")
(setf (get kortex_driver::GetArmState :datatype-) "kortex_driver/GetArmState")
(setf (get kortex_driver::GetArmState :request) kortex_driver::GetArmStateRequest)
(setf (get kortex_driver::GetArmState :response) kortex_driver::GetArmStateResponse)

(defmethod kortex_driver::GetArmStateRequest
  (:response () (instance kortex_driver::GetArmStateResponse :init)))

(setf (get kortex_driver::GetArmStateRequest :md5sum-) "09192f2d3f6462e1466a4468f2c2390a")
(setf (get kortex_driver::GetArmStateRequest :datatype-) "kortex_driver/GetArmStateRequest")
(setf (get kortex_driver::GetArmStateRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ArmStateInformation output

================================================================================
MSG: kortex_driver/ArmStateInformation

uint32 active_state
Connection connection
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")

(setf (get kortex_driver::GetArmStateResponse :md5sum-) "09192f2d3f6462e1466a4468f2c2390a")
(setf (get kortex_driver::GetArmStateResponse :datatype-) "kortex_driver/GetArmStateResponse")
(setf (get kortex_driver::GetArmStateResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ArmStateInformation output

================================================================================
MSG: kortex_driver/ArmStateInformation

uint32 active_state
Connection connection
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")



(provide :kortex_driver/GetArmState "09192f2d3f6462e1466a4468f2c2390a")


