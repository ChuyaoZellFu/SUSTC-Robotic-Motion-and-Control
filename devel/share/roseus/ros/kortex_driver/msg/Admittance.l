;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Admittance)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Admittance (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ADMITTANCE")
  (make-package "KORTEX_DRIVER::ADMITTANCE"))

(in-package "ROS")
;;//! \htmlinclude Admittance.msg.html


(defclass kortex_driver::Admittance
  :super ros::object
  :slots (_admittance_mode ))

(defmethod kortex_driver::Admittance
  (:init
   (&key
    ((:admittance_mode __admittance_mode) 0)
    )
   (send-super :init)
   (setq _admittance_mode (round __admittance_mode))
   self)
  (:admittance_mode
   (&optional __admittance_mode)
   (if __admittance_mode (setq _admittance_mode __admittance_mode)) _admittance_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _admittance_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _admittance_mode
       (write-long _admittance_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _admittance_mode
     (setq _admittance_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Admittance :md5sum-) "5f17929e02863c928199e8741332cc24")
(setf (get kortex_driver::Admittance :datatype-) "kortex_driver/Admittance")
(setf (get kortex_driver::Admittance :definition-)
      "
uint32 admittance_mode
")



(provide :kortex_driver/Admittance "5f17929e02863c928199e8741332cc24")


