;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ApiOptions)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ApiOptions (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::APIOPTIONS")
  (make-package "KORTEX_DRIVER::APIOPTIONS"))

(in-package "ROS")
;;//! \htmlinclude ApiOptions.msg.html


(defclass kortex_driver::ApiOptions
  :super ros::object
  :slots (_timeout_ms ))

(defmethod kortex_driver::ApiOptions
  (:init
   (&key
    ((:timeout_ms __timeout_ms) 0)
    )
   (send-super :init)
   (setq _timeout_ms (round __timeout_ms))
   self)
  (:timeout_ms
   (&optional __timeout_ms)
   (if __timeout_ms (setq _timeout_ms __timeout_ms)) _timeout_ms)
  (:serialization-length
   ()
   (+
    ;; uint32 _timeout_ms
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _timeout_ms
       (write-long _timeout_ms s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _timeout_ms
     (setq _timeout_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ApiOptions :md5sum-) "88b8e888a6c1e4d612523746429635b7")
(setf (get kortex_driver::ApiOptions :datatype-) "kortex_driver/ApiOptions")
(setf (get kortex_driver::ApiOptions :definition-)
      "uint32 timeout_ms
")



(provide :kortex_driver/ApiOptions "88b8e888a6c1e4d612523746429635b7")


