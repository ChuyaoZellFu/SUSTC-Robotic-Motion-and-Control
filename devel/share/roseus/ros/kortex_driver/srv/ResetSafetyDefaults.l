;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetSafetyDefaults)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetSafetyDefaults (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETSAFETYDEFAULTS")
  (make-package "KORTEX_DRIVER::RESETSAFETYDEFAULTS"))
(unless (find-package "KORTEX_DRIVER::RESETSAFETYDEFAULTSREQUEST")
  (make-package "KORTEX_DRIVER::RESETSAFETYDEFAULTSREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETSAFETYDEFAULTSRESPONSE")
  (make-package "KORTEX_DRIVER::RESETSAFETYDEFAULTSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetSafetyDefaultsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetSafetyDefaultsRequest
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

(defclass kortex_driver::ResetSafetyDefaultsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetSafetyDefaultsResponse
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

(defclass kortex_driver::ResetSafetyDefaults
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetSafetyDefaults :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResetSafetyDefaults :datatype-) "kortex_driver/ResetSafetyDefaults")
(setf (get kortex_driver::ResetSafetyDefaults :request) kortex_driver::ResetSafetyDefaultsRequest)
(setf (get kortex_driver::ResetSafetyDefaults :response) kortex_driver::ResetSafetyDefaultsResponse)

(defmethod kortex_driver::ResetSafetyDefaultsRequest
  (:response () (instance kortex_driver::ResetSafetyDefaultsResponse :init)))

(setf (get kortex_driver::ResetSafetyDefaultsRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResetSafetyDefaultsRequest :datatype-) "kortex_driver/ResetSafetyDefaultsRequest")
(setf (get kortex_driver::ResetSafetyDefaultsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ResetSafetyDefaultsResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResetSafetyDefaultsResponse :datatype-) "kortex_driver/ResetSafetyDefaultsResponse")
(setf (get kortex_driver::ResetSafetyDefaultsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ResetSafetyDefaults "f335b819dc59099fe3124b36f140ad07")


