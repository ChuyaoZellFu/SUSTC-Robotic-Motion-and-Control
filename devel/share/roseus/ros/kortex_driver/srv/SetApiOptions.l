;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetApiOptions)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetApiOptions (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETAPIOPTIONS")
  (make-package "KORTEX_DRIVER::SETAPIOPTIONS"))
(unless (find-package "KORTEX_DRIVER::SETAPIOPTIONSREQUEST")
  (make-package "KORTEX_DRIVER::SETAPIOPTIONSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETAPIOPTIONSRESPONSE")
  (make-package "KORTEX_DRIVER::SETAPIOPTIONSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetApiOptionsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetApiOptionsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ApiOptions :init))
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
    ;; kortex_driver/ApiOptions _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ApiOptions _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ApiOptions _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetApiOptionsResponse
  :super ros::object
  :slots ())

(defmethod kortex_driver::SetApiOptionsResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass kortex_driver::SetApiOptions
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetApiOptions :md5sum-) "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(setf (get kortex_driver::SetApiOptions :datatype-) "kortex_driver/SetApiOptions")
(setf (get kortex_driver::SetApiOptions :request) kortex_driver::SetApiOptionsRequest)
(setf (get kortex_driver::SetApiOptions :response) kortex_driver::SetApiOptionsResponse)

(defmethod kortex_driver::SetApiOptionsRequest
  (:response () (instance kortex_driver::SetApiOptionsResponse :init)))

(setf (get kortex_driver::SetApiOptionsRequest :md5sum-) "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(setf (get kortex_driver::SetApiOptionsRequest :datatype-) "kortex_driver/SetApiOptionsRequest")
(setf (get kortex_driver::SetApiOptionsRequest :definition-)
      "ApiOptions input

================================================================================
MSG: kortex_driver/ApiOptions
---


")

(setf (get kortex_driver::SetApiOptionsResponse :md5sum-) "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(setf (get kortex_driver::SetApiOptionsResponse :datatype-) "kortex_driver/SetApiOptionsResponse")
(setf (get kortex_driver::SetApiOptionsResponse :definition-)
      "ApiOptions input

================================================================================
MSG: kortex_driver/ApiOptions
---


")



(provide :kortex_driver/SetApiOptions "11c85b567b00c6eb6cd55fdb8cb9ad1b")


