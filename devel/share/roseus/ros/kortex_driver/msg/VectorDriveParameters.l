;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::VectorDriveParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'VectorDriveParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::VECTORDRIVEPARAMETERS")
  (make-package "KORTEX_DRIVER::VECTORDRIVEPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude VectorDriveParameters.msg.html


(defclass kortex_driver::VectorDriveParameters
  :super ros::object
  :slots (_kpq _kiq _kpd _kid ))

(defmethod kortex_driver::VectorDriveParameters
  (:init
   (&key
    ((:kpq __kpq) 0.0)
    ((:kiq __kiq) 0.0)
    ((:kpd __kpd) 0.0)
    ((:kid __kid) 0.0)
    )
   (send-super :init)
   (setq _kpq (float __kpq))
   (setq _kiq (float __kiq))
   (setq _kpd (float __kpd))
   (setq _kid (float __kid))
   self)
  (:kpq
   (&optional __kpq)
   (if __kpq (setq _kpq __kpq)) _kpq)
  (:kiq
   (&optional __kiq)
   (if __kiq (setq _kiq __kiq)) _kiq)
  (:kpd
   (&optional __kpd)
   (if __kpd (setq _kpd __kpd)) _kpd)
  (:kid
   (&optional __kid)
   (if __kid (setq _kid __kid)) _kid)
  (:serialization-length
   ()
   (+
    ;; float32 _kpq
    4
    ;; float32 _kiq
    4
    ;; float32 _kpd
    4
    ;; float32 _kid
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _kpq
       (sys::poke _kpq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kiq
       (sys::poke _kiq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kpd
       (sys::poke _kpd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kid
       (sys::poke _kid (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _kpq
     (setq _kpq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kiq
     (setq _kiq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kpd
     (setq _kpd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kid
     (setq _kid (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::VectorDriveParameters :md5sum-) "30e295016ac63da10cbe59e969bb6401")
(setf (get kortex_driver::VectorDriveParameters :datatype-) "kortex_driver/VectorDriveParameters")
(setf (get kortex_driver::VectorDriveParameters :definition-)
      "
float32 kpq
float32 kiq
float32 kpd
float32 kid
")



(provide :kortex_driver/VectorDriveParameters "30e295016ac63da10cbe59e969bb6401")


