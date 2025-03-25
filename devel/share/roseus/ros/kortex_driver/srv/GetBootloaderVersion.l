;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetBootloaderVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetBootloaderVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETBOOTLOADERVERSION")
  (make-package "KORTEX_DRIVER::GETBOOTLOADERVERSION"))
(unless (find-package "KORTEX_DRIVER::GETBOOTLOADERVERSIONREQUEST")
  (make-package "KORTEX_DRIVER::GETBOOTLOADERVERSIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETBOOTLOADERVERSIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETBOOTLOADERVERSIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetBootloaderVersionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetBootloaderVersionRequest
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

(defclass kortex_driver::GetBootloaderVersionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetBootloaderVersionResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::BootloaderVersion :init))
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
    ;; kortex_driver/BootloaderVersion _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BootloaderVersion _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BootloaderVersion _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBootloaderVersion
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetBootloaderVersion :md5sum-) "e674f344ecc5f8e12f3233bf3b7cbeaa")
(setf (get kortex_driver::GetBootloaderVersion :datatype-) "kortex_driver/GetBootloaderVersion")
(setf (get kortex_driver::GetBootloaderVersion :request) kortex_driver::GetBootloaderVersionRequest)
(setf (get kortex_driver::GetBootloaderVersion :response) kortex_driver::GetBootloaderVersionResponse)

(defmethod kortex_driver::GetBootloaderVersionRequest
  (:response () (instance kortex_driver::GetBootloaderVersionResponse :init)))

(setf (get kortex_driver::GetBootloaderVersionRequest :md5sum-) "e674f344ecc5f8e12f3233bf3b7cbeaa")
(setf (get kortex_driver::GetBootloaderVersionRequest :datatype-) "kortex_driver/GetBootloaderVersionRequest")
(setf (get kortex_driver::GetBootloaderVersionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BootloaderVersion output

================================================================================
MSG: kortex_driver/BootloaderVersion

")

(setf (get kortex_driver::GetBootloaderVersionResponse :md5sum-) "e674f344ecc5f8e12f3233bf3b7cbeaa")
(setf (get kortex_driver::GetBootloaderVersionResponse :datatype-) "kortex_driver/GetBootloaderVersionResponse")
(setf (get kortex_driver::GetBootloaderVersionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BootloaderVersion output

================================================================================
MSG: kortex_driver/BootloaderVersion

")



(provide :kortex_driver/GetBootloaderVersion "e674f344ecc5f8e12f3233bf3b7cbeaa")


