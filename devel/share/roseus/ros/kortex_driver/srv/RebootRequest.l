;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RebootRequest)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RebootRequest (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::REBOOTREQUEST")
  (make-package "KORTEX_DRIVER::REBOOTREQUEST"))
(unless (find-package "KORTEX_DRIVER::REBOOTREQUESTREQUEST")
  (make-package "KORTEX_DRIVER::REBOOTREQUESTREQUEST"))
(unless (find-package "KORTEX_DRIVER::REBOOTREQUESTRESPONSE")
  (make-package "KORTEX_DRIVER::REBOOTREQUESTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::RebootRequestRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::RebootRequestRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::RebootRqst :init))
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
    ;; kortex_driver/RebootRqst _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/RebootRqst _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/RebootRqst _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::RebootRequestResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::RebootRequestResponse
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

(defclass kortex_driver::RebootRequest
  :super ros::object
  :slots ())

(setf (get kortex_driver::RebootRequest :md5sum-) "7587d0460f4d1dff902eaf7ca6589a94")
(setf (get kortex_driver::RebootRequest :datatype-) "kortex_driver/RebootRequest")
(setf (get kortex_driver::RebootRequest :request) kortex_driver::RebootRequestRequest)
(setf (get kortex_driver::RebootRequest :response) kortex_driver::RebootRequestResponse)

(defmethod kortex_driver::RebootRequestRequest
  (:response () (instance kortex_driver::RebootRequestResponse :init)))

(setf (get kortex_driver::RebootRequestRequest :md5sum-) "7587d0460f4d1dff902eaf7ca6589a94")
(setf (get kortex_driver::RebootRequestRequest :datatype-) "kortex_driver/RebootRequestRequest")
(setf (get kortex_driver::RebootRequestRequest :definition-)
      "RebootRqst input

================================================================================
MSG: kortex_driver/RebootRqst

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::RebootRequestResponse :md5sum-) "7587d0460f4d1dff902eaf7ca6589a94")
(setf (get kortex_driver::RebootRequestResponse :datatype-) "kortex_driver/RebootRequestResponse")
(setf (get kortex_driver::RebootRequestResponse :definition-)
      "RebootRqst input

================================================================================
MSG: kortex_driver/RebootRqst

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/RebootRequest "7587d0460f4d1dff902eaf7ca6589a94")


