;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_GetCapSenseConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_GetCapSenseConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIG")
  (make-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIG"))
(unless (find-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIGREQUEST")
  (make-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIGREQUEST"))
(unless (find-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIGRESPONSE")
  (make-package "KORTEX_DRIVER::BASE_GETCAPSENSECONFIGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::Base_GetCapSenseConfigRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::Base_GetCapSenseConfigRequest
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

(defclass kortex_driver::Base_GetCapSenseConfigResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::Base_GetCapSenseConfigResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Base_CapSenseConfig :init))
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
    ;; kortex_driver/Base_CapSenseConfig _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_CapSenseConfig _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_CapSenseConfig _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::Base_GetCapSenseConfig
  :super ros::object
  :slots ())

(setf (get kortex_driver::Base_GetCapSenseConfig :md5sum-) "c1e479a50a0c96ec49d735b61e1b4a43")
(setf (get kortex_driver::Base_GetCapSenseConfig :datatype-) "kortex_driver/Base_GetCapSenseConfig")
(setf (get kortex_driver::Base_GetCapSenseConfig :request) kortex_driver::Base_GetCapSenseConfigRequest)
(setf (get kortex_driver::Base_GetCapSenseConfig :response) kortex_driver::Base_GetCapSenseConfigResponse)

(defmethod kortex_driver::Base_GetCapSenseConfigRequest
  (:response () (instance kortex_driver::Base_GetCapSenseConfigResponse :init)))

(setf (get kortex_driver::Base_GetCapSenseConfigRequest :md5sum-) "c1e479a50a0c96ec49d735b61e1b4a43")
(setf (get kortex_driver::Base_GetCapSenseConfigRequest :datatype-) "kortex_driver/Base_GetCapSenseConfigRequest")
(setf (get kortex_driver::Base_GetCapSenseConfigRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Base_CapSenseConfig output

================================================================================
MSG: kortex_driver/Base_CapSenseConfig

uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
")

(setf (get kortex_driver::Base_GetCapSenseConfigResponse :md5sum-) "c1e479a50a0c96ec49d735b61e1b4a43")
(setf (get kortex_driver::Base_GetCapSenseConfigResponse :datatype-) "kortex_driver/Base_GetCapSenseConfigResponse")
(setf (get kortex_driver::Base_GetCapSenseConfigResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Base_CapSenseConfig output

================================================================================
MSG: kortex_driver/Base_CapSenseConfig

uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
")



(provide :kortex_driver/Base_GetCapSenseConfig "c1e479a50a0c96ec49d735b61e1b4a43")


