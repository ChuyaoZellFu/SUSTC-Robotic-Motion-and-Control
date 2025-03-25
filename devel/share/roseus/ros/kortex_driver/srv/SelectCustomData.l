;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SelectCustomData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SelectCustomData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SELECTCUSTOMDATA")
  (make-package "KORTEX_DRIVER::SELECTCUSTOMDATA"))
(unless (find-package "KORTEX_DRIVER::SELECTCUSTOMDATAREQUEST")
  (make-package "KORTEX_DRIVER::SELECTCUSTOMDATAREQUEST"))
(unless (find-package "KORTEX_DRIVER::SELECTCUSTOMDATARESPONSE")
  (make-package "KORTEX_DRIVER::SELECTCUSTOMDATARESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SelectCustomDataRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SelectCustomDataRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CustomDataSelection :init))
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
    ;; kortex_driver/CustomDataSelection _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CustomDataSelection _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CustomDataSelection _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SelectCustomDataResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SelectCustomDataResponse
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

(defclass kortex_driver::SelectCustomData
  :super ros::object
  :slots ())

(setf (get kortex_driver::SelectCustomData :md5sum-) "6fbafd6efc00928eb53e0c769fef2624")
(setf (get kortex_driver::SelectCustomData :datatype-) "kortex_driver/SelectCustomData")
(setf (get kortex_driver::SelectCustomData :request) kortex_driver::SelectCustomDataRequest)
(setf (get kortex_driver::SelectCustomData :response) kortex_driver::SelectCustomDataResponse)

(defmethod kortex_driver::SelectCustomDataRequest
  (:response () (instance kortex_driver::SelectCustomDataResponse :init)))

(setf (get kortex_driver::SelectCustomDataRequest :md5sum-) "6fbafd6efc00928eb53e0c769fef2624")
(setf (get kortex_driver::SelectCustomDataRequest :datatype-) "kortex_driver/SelectCustomDataRequest")
(setf (get kortex_driver::SelectCustomDataRequest :definition-)
      "CustomDataSelection input

================================================================================
MSG: kortex_driver/CustomDataSelection

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SelectCustomDataResponse :md5sum-) "6fbafd6efc00928eb53e0c769fef2624")
(setf (get kortex_driver::SelectCustomDataResponse :datatype-) "kortex_driver/SelectCustomDataResponse")
(setf (get kortex_driver::SelectCustomDataResponse :definition-)
      "CustomDataSelection input

================================================================================
MSG: kortex_driver/CustomDataSelection

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SelectCustomData "6fbafd6efc00928eb53e0c769fef2624")


