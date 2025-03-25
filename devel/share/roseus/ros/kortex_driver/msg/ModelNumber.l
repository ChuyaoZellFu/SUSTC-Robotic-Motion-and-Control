;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ModelNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ModelNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MODELNUMBER")
  (make-package "KORTEX_DRIVER::MODELNUMBER"))

(in-package "ROS")
;;//! \htmlinclude ModelNumber.msg.html


(defclass kortex_driver::ModelNumber
  :super ros::object
  :slots (_model_number ))

(defmethod kortex_driver::ModelNumber
  (:init
   (&key
    ((:model_number __model_number) "")
    )
   (send-super :init)
   (setq _model_number (string __model_number))
   self)
  (:model_number
   (&optional __model_number)
   (if __model_number (setq _model_number __model_number)) _model_number)
  (:serialization-length
   ()
   (+
    ;; string _model_number
    4 (length _model_number)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_number
       (write-long (length _model_number) s) (princ _model_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::ModelNumber :md5sum-) "f12cb4b23059487b453a88e8d41162b5")
(setf (get kortex_driver::ModelNumber :datatype-) "kortex_driver/ModelNumber")
(setf (get kortex_driver::ModelNumber :definition-)
      "
string model_number
")



(provide :kortex_driver/ModelNumber "f12cb4b23059487b453a88e8d41162b5")


