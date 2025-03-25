;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TransformationRow)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TransformationRow (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRANSFORMATIONROW")
  (make-package "KORTEX_DRIVER::TRANSFORMATIONROW"))

(in-package "ROS")
;;//! \htmlinclude TransformationRow.msg.html


(defclass kortex_driver::TransformationRow
  :super ros::object
  :slots (_c0 _c1 _c2 _c3 ))

(defmethod kortex_driver::TransformationRow
  (:init
   (&key
    ((:c0 __c0) 0.0)
    ((:c1 __c1) 0.0)
    ((:c2 __c2) 0.0)
    ((:c3 __c3) 0.0)
    )
   (send-super :init)
   (setq _c0 (float __c0))
   (setq _c1 (float __c1))
   (setq _c2 (float __c2))
   (setq _c3 (float __c3))
   self)
  (:c0
   (&optional __c0)
   (if __c0 (setq _c0 __c0)) _c0)
  (:c1
   (&optional __c1)
   (if __c1 (setq _c1 __c1)) _c1)
  (:c2
   (&optional __c2)
   (if __c2 (setq _c2 __c2)) _c2)
  (:c3
   (&optional __c3)
   (if __c3 (setq _c3 __c3)) _c3)
  (:serialization-length
   ()
   (+
    ;; float32 _c0
    4
    ;; float32 _c1
    4
    ;; float32 _c2
    4
    ;; float32 _c3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _c0
       (sys::poke _c0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c1
       (sys::poke _c1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c2
       (sys::poke _c2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c3
       (sys::poke _c3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _c0
     (setq _c0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c1
     (setq _c1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c2
     (setq _c2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c3
     (setq _c3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TransformationRow :md5sum-) "0b9e4dcf087a9f1dd44881ce6cb761b8")
(setf (get kortex_driver::TransformationRow :datatype-) "kortex_driver/TransformationRow")
(setf (get kortex_driver::TransformationRow :definition-)
      "
float32 c0
float32 c1
float32 c2
float32 c3
")



(provide :kortex_driver/TransformationRow "0b9e4dcf087a9f1dd44881ce6cb761b8")


