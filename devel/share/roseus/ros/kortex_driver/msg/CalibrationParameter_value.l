;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationParameter_value)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationParameter_value (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONPARAMETER_VALUE")
  (make-package "KORTEX_DRIVER::CALIBRATIONPARAMETER_VALUE"))

(in-package "ROS")
;;//! \htmlinclude CalibrationParameter_value.msg.html


(defclass kortex_driver::CalibrationParameter_value
  :super ros::object
  :slots (_signedIntValue _unsignedIntValue _floatValue ))

(defmethod kortex_driver::CalibrationParameter_value
  (:init
   (&key
    ((:signedIntValue __signedIntValue) (make-array 0 :initial-element 0 :element-type :integer))
    ((:unsignedIntValue __unsignedIntValue) (make-array 0 :initial-element 0 :element-type :integer))
    ((:floatValue __floatValue) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _signedIntValue __signedIntValue)
   (setq _unsignedIntValue __unsignedIntValue)
   (setq _floatValue __floatValue)
   self)
  (:signedIntValue
   (&optional __signedIntValue)
   (if __signedIntValue (setq _signedIntValue __signedIntValue)) _signedIntValue)
  (:unsignedIntValue
   (&optional __unsignedIntValue)
   (if __unsignedIntValue (setq _unsignedIntValue __unsignedIntValue)) _unsignedIntValue)
  (:floatValue
   (&optional __floatValue)
   (if __floatValue (setq _floatValue __floatValue)) _floatValue)
  (:serialization-length
   ()
   (+
    ;; uint32[] _signedIntValue
    (* 4    (length _signedIntValue)) 4
    ;; uint32[] _unsignedIntValue
    (* 4    (length _unsignedIntValue)) 4
    ;; uint32[] _floatValue
    (* 4    (length _floatValue)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _signedIntValue
     (write-long (length _signedIntValue) s)
     (dotimes (i (length _signedIntValue))
       (write-long (elt _signedIntValue i) s)
       )
     ;; uint32[] _unsignedIntValue
     (write-long (length _unsignedIntValue) s)
     (dotimes (i (length _unsignedIntValue))
       (write-long (elt _unsignedIntValue i) s)
       )
     ;; uint32[] _floatValue
     (write-long (length _floatValue) s)
     (dotimes (i (length _floatValue))
       (write-long (elt _floatValue i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _signedIntValue
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _signedIntValue (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _signedIntValue i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _unsignedIntValue
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _unsignedIntValue (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _unsignedIntValue i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _floatValue
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _floatValue (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _floatValue i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::CalibrationParameter_value :md5sum-) "2b45c1a772c79ef326799a208a383734")
(setf (get kortex_driver::CalibrationParameter_value :datatype-) "kortex_driver/CalibrationParameter_value")
(setf (get kortex_driver::CalibrationParameter_value :definition-)
      "
uint32[] signedIntValue
uint32[] unsignedIntValue
uint32[] floatValue
")



(provide :kortex_driver/CalibrationParameter_value "2b45c1a772c79ef326799a208a383734")


