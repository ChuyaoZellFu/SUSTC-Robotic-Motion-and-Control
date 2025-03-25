;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CountryCode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CountryCode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COUNTRYCODE")
  (make-package "KORTEX_DRIVER::COUNTRYCODE"))

(in-package "ROS")
;;//! \htmlinclude CountryCode.msg.html


(defclass kortex_driver::CountryCode
  :super ros::object
  :slots (_identifier ))

(defmethod kortex_driver::CountryCode
  (:init
   (&key
    ((:identifier __identifier) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _identifier
       (write-long _identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CountryCode :md5sum-) "8819d354d8745a64b42def74f5c8102a")
(setf (get kortex_driver::CountryCode :datatype-) "kortex_driver/CountryCode")
(setf (get kortex_driver::CountryCode :definition-)
      "
uint32 identifier
")



(provide :kortex_driver/CountryCode "8819d354d8745a64b42def74f5c8102a")


