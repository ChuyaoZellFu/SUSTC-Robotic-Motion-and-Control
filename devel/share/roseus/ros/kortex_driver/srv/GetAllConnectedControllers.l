;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllConnectedControllers)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllConnectedControllers (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERS")
  (make-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERS"))
(unless (find-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERSREQUEST")
  (make-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERSRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLCONNECTEDCONTROLLERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllConnectedControllersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllConnectedControllersRequest
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

(defclass kortex_driver::GetAllConnectedControllersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllConnectedControllersResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControllerList :init))
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
    ;; kortex_driver/ControllerList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllConnectedControllers
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllConnectedControllers :md5sum-) "cf23ee5904b15aa6ac88d84a59fc18f9")
(setf (get kortex_driver::GetAllConnectedControllers :datatype-) "kortex_driver/GetAllConnectedControllers")
(setf (get kortex_driver::GetAllConnectedControllers :request) kortex_driver::GetAllConnectedControllersRequest)
(setf (get kortex_driver::GetAllConnectedControllers :response) kortex_driver::GetAllConnectedControllersResponse)

(defmethod kortex_driver::GetAllConnectedControllersRequest
  (:response () (instance kortex_driver::GetAllConnectedControllersResponse :init)))

(setf (get kortex_driver::GetAllConnectedControllersRequest :md5sum-) "cf23ee5904b15aa6ac88d84a59fc18f9")
(setf (get kortex_driver::GetAllConnectedControllersRequest :datatype-) "kortex_driver/GetAllConnectedControllersRequest")
(setf (get kortex_driver::GetAllConnectedControllersRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerList output

================================================================================
MSG: kortex_driver/ControllerList

ControllerHandle[] handles
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
")

(setf (get kortex_driver::GetAllConnectedControllersResponse :md5sum-) "cf23ee5904b15aa6ac88d84a59fc18f9")
(setf (get kortex_driver::GetAllConnectedControllersResponse :datatype-) "kortex_driver/GetAllConnectedControllersResponse")
(setf (get kortex_driver::GetAllConnectedControllersResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerList output

================================================================================
MSG: kortex_driver/ControllerList

ControllerHandle[] handles
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
")



(provide :kortex_driver/GetAllConnectedControllers "cf23ee5904b15aa6ac88d84a59fc18f9")


