;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianSpeed)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianSpeed (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANSPEED")
  (make-package "KORTEX_DRIVER::CARTESIANSPEED"))

(in-package "ROS")
;;//! \htmlinclude CartesianSpeed.msg.html


(defclass kortex_driver::CartesianSpeed
  :super ros::object
  :slots (_translation _orientation ))

(defmethod kortex_driver::CartesianSpeed
  (:init
   (&key
    ((:translation __translation) 0.0)
    ((:orientation __orientation) 0.0)
    )
   (send-super :init)
   (setq _translation (float __translation))
   (setq _orientation (float __orientation))
   self)
  (:translation
   (&optional __translation)
   (if __translation (setq _translation __translation)) _translation)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:serialization-length
   ()
   (+
    ;; float32 _translation
    4
    ;; float32 _orientation
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _translation
       (sys::poke _translation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientation
       (sys::poke _orientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _translation
     (setq _translation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientation
     (setq _orientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CartesianSpeed :md5sum-) "0fa17c7fbcac20ea2a06fae93c7ef126")
(setf (get kortex_driver::CartesianSpeed :datatype-) "kortex_driver/CartesianSpeed")
(setf (get kortex_driver::CartesianSpeed :definition-)
      "
float32 translation
float32 orientation
")



(provide :kortex_driver/CartesianSpeed "0fa17c7fbcac20ea2a06fae93c7ef126")


