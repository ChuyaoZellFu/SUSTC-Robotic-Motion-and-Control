;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Ssid)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Ssid (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SSID")
  (make-package "KORTEX_DRIVER::SSID"))

(in-package "ROS")
;;//! \htmlinclude Ssid.msg.html


(defclass kortex_driver::Ssid
  :super ros::object
  :slots (_identifier ))

(defmethod kortex_driver::Ssid
  (:init
   (&key
    ((:identifier __identifier) "")
    )
   (send-super :init)
   (setq _identifier (string __identifier))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:serialization-length
   ()
   (+
    ;; string _identifier
    4 (length _identifier)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _identifier
       (write-long (length _identifier) s) (princ _identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _identifier
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _identifier (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::Ssid :md5sum-) "9c4b0f781baa7fd49cc9e186f2f56898")
(setf (get kortex_driver::Ssid :datatype-) "kortex_driver/Ssid")
(setf (get kortex_driver::Ssid :definition-)
      "
string identifier
")



(provide :kortex_driver/Ssid "9c4b0f781baa7fd49cc9e186f2f56898")


