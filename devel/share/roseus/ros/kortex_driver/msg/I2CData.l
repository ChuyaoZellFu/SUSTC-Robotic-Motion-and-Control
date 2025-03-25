;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CDATA")
  (make-package "KORTEX_DRIVER::I2CDATA"))

(in-package "ROS")
;;//! \htmlinclude I2CData.msg.html


(defclass kortex_driver::I2CData
  :super ros::object
  :slots (_data _size ))

(defmethod kortex_driver::I2CData
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    ((:size __size) 0)
    )
   (send-super :init)
   (setq _data __data)
   (setq _size (round __size))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ;; uint32 _size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;; uint32 _size
       (write-long _size s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::I2CData :md5sum-) "3abc3c9d46f09b6c612dbaa8aeebf087")
(setf (get kortex_driver::I2CData :datatype-) "kortex_driver/I2CData")
(setf (get kortex_driver::I2CData :definition-)
      "
uint8[] data
uint32 size
")



(provide :kortex_driver/I2CData "3abc3c9d46f09b6c612dbaa8aeebf087")


