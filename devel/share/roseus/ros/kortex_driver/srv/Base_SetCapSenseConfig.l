;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_SetCapSenseConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_SetCapSenseConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIG")
  (make-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIG"))
(unless (find-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIGREQUEST")
  (make-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIGREQUEST"))
(unless (find-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIGRESPONSE")
  (make-package "KORTEX_DRIVER::BASE_SETCAPSENSECONFIGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::Base_SetCapSenseConfigRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::Base_SetCapSenseConfigRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Base_CapSenseConfig :init))
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
    ;; kortex_driver/Base_CapSenseConfig _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_CapSenseConfig _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_CapSenseConfig _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::Base_SetCapSenseConfigResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::Base_SetCapSenseConfigResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::Base_SetCapSenseConfig
  :super ros::object
  :slots ())

(setf (get kortex_driver::Base_SetCapSenseConfig :md5sum-) "8ed9ca8d37eb83b52505e6684b7dc6f8")
(setf (get kortex_driver::Base_SetCapSenseConfig :datatype-) "kortex_driver/Base_SetCapSenseConfig")
(setf (get kortex_driver::Base_SetCapSenseConfig :request) kortex_driver::Base_SetCapSenseConfigRequest)
(setf (get kortex_driver::Base_SetCapSenseConfig :response) kortex_driver::Base_SetCapSenseConfigResponse)

(defmethod kortex_driver::Base_SetCapSenseConfigRequest
  (:response () (instance kortex_driver::Base_SetCapSenseConfigResponse :init)))

(setf (get kortex_driver::Base_SetCapSenseConfigRequest :md5sum-) "8ed9ca8d37eb83b52505e6684b7dc6f8")
(setf (get kortex_driver::Base_SetCapSenseConfigRequest :datatype-) "kortex_driver/Base_SetCapSenseConfigRequest")
(setf (get kortex_driver::Base_SetCapSenseConfigRequest :definition-)
      "Base_CapSenseConfig input

================================================================================
MSG: kortex_driver/Base_CapSenseConfig

uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::Base_SetCapSenseConfigResponse :md5sum-) "8ed9ca8d37eb83b52505e6684b7dc6f8")
(setf (get kortex_driver::Base_SetCapSenseConfigResponse :datatype-) "kortex_driver/Base_SetCapSenseConfigResponse")
(setf (get kortex_driver::Base_SetCapSenseConfigResponse :definition-)
      "Base_CapSenseConfig input

================================================================================
MSG: kortex_driver/Base_CapSenseConfig

uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/Base_SetCapSenseConfig "8ed9ca8d37eb83b52505e6684b7dc6f8")


