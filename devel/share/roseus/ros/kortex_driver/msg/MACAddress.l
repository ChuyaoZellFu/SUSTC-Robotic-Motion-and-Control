;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MACAddress)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MACAddress (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MACADDRESS")
  (make-package "KORTEX_DRIVER::MACADDRESS"))

(in-package "ROS")
;;//! \htmlinclude MACAddress.msg.html


(defclass kortex_driver::MACAddress
  :super ros::object
  :slots (_mac_address ))

(defmethod kortex_driver::MACAddress
  (:init
   (&key
    ((:mac_address __mac_address) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _mac_address __mac_address)
   self)
  (:mac_address
   (&optional __mac_address)
   (if __mac_address (setq _mac_address __mac_address)) _mac_address)
  (:serialization-length
   ()
   (+
    ;; uint8[] _mac_address
    (* 1    (length _mac_address)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _mac_address
     (write-long (length _mac_address) s)
     (princ _mac_address s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _mac_address
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _mac_address (make-array n :element-type :char))
     (replace _mac_address buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::MACAddress :md5sum-) "5d1c55e7218bc267e43e7230dde48c16")
(setf (get kortex_driver::MACAddress :datatype-) "kortex_driver/MACAddress")
(setf (get kortex_driver::MACAddress :definition-)
      "
uint8[] mac_address
")



(provide :kortex_driver/MACAddress "5d1c55e7218bc267e43e7230dde48c16")


