;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SerialNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SerialNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SERIALNUMBER")
  (make-package "KORTEX_DRIVER::SERIALNUMBER"))

(in-package "ROS")
;;//! \htmlinclude SerialNumber.msg.html


(defclass kortex_driver::SerialNumber
  :super ros::object
  :slots (_serial_number ))

(defmethod kortex_driver::SerialNumber
  (:init
   (&key
    ((:serial_number __serial_number) "")
    )
   (send-super :init)
   (setq _serial_number (string __serial_number))
   self)
  (:serial_number
   (&optional __serial_number)
   (if __serial_number (setq _serial_number __serial_number)) _serial_number)
  (:serialization-length
   ()
   (+
    ;; string _serial_number
    4 (length _serial_number)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _serial_number
       (write-long (length _serial_number) s) (princ _serial_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _serial_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _serial_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::SerialNumber :md5sum-) "fb68edee72c8891f2105e89acba723ea")
(setf (get kortex_driver::SerialNumber :datatype-) "kortex_driver/SerialNumber")
(setf (get kortex_driver::SerialNumber :definition-)
      "
string serial_number
")



(provide :kortex_driver/SerialNumber "fb68edee72c8891f2105e89acba723ea")


