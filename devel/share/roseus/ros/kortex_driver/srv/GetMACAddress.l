;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetMACAddress)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetMACAddress (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETMACADDRESS")
  (make-package "KORTEX_DRIVER::GETMACADDRESS"))
(unless (find-package "KORTEX_DRIVER::GETMACADDRESSREQUEST")
  (make-package "KORTEX_DRIVER::GETMACADDRESSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETMACADDRESSRESPONSE")
  (make-package "KORTEX_DRIVER::GETMACADDRESSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetMACAddressRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetMACAddressRequest
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

(defclass kortex_driver::GetMACAddressResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetMACAddressResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::MACAddress :init))
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
    ;; kortex_driver/MACAddress _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MACAddress _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MACAddress _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetMACAddress
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetMACAddress :md5sum-) "bf0ce987b74a5e470bf1deeae1772263")
(setf (get kortex_driver::GetMACAddress :datatype-) "kortex_driver/GetMACAddress")
(setf (get kortex_driver::GetMACAddress :request) kortex_driver::GetMACAddressRequest)
(setf (get kortex_driver::GetMACAddress :response) kortex_driver::GetMACAddressResponse)

(defmethod kortex_driver::GetMACAddressRequest
  (:response () (instance kortex_driver::GetMACAddressResponse :init)))

(setf (get kortex_driver::GetMACAddressRequest :md5sum-) "bf0ce987b74a5e470bf1deeae1772263")
(setf (get kortex_driver::GetMACAddressRequest :datatype-) "kortex_driver/GetMACAddressRequest")
(setf (get kortex_driver::GetMACAddressRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
MACAddress output

================================================================================
MSG: kortex_driver/MACAddress

")

(setf (get kortex_driver::GetMACAddressResponse :md5sum-) "bf0ce987b74a5e470bf1deeae1772263")
(setf (get kortex_driver::GetMACAddressResponse :datatype-) "kortex_driver/GetMACAddressResponse")
(setf (get kortex_driver::GetMACAddressResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
MACAddress output

================================================================================
MSG: kortex_driver/MACAddress

")



(provide :kortex_driver/GetMACAddress "bf0ce987b74a5e470bf1deeae1772263")


