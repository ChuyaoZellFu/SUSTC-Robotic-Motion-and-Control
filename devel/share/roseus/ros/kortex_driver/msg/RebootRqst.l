;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RebootRqst)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RebootRqst (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::REBOOTRQST")
  (make-package "KORTEX_DRIVER::REBOOTRQST"))

(in-package "ROS")
;;//! \htmlinclude RebootRqst.msg.html


(defclass kortex_driver::RebootRqst
  :super ros::object
  :slots (_delay ))

(defmethod kortex_driver::RebootRqst
  (:init
   (&key
    ((:delay __delay) 0)
    )
   (send-super :init)
   (setq _delay (round __delay))
   self)
  (:delay
   (&optional __delay)
   (if __delay (setq _delay __delay)) _delay)
  (:serialization-length
   ()
   (+
    ;; uint32 _delay
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _delay
       (write-long _delay s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _delay
     (setq _delay (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::RebootRqst :md5sum-) "5759c6917ead1447f445ebc594ffe2ec")
(setf (get kortex_driver::RebootRqst :datatype-) "kortex_driver/RebootRqst")
(setf (get kortex_driver::RebootRqst :definition-)
      "
uint32 delay
")



(provide :kortex_driver/RebootRqst "5759c6917ead1447f445ebc594ffe2ec")


