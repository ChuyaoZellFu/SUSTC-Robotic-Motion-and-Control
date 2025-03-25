;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_RotationMatrixRow)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_RotationMatrixRow (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_ROTATIONMATRIXROW")
  (make-package "KORTEX_DRIVER::BASE_ROTATIONMATRIXROW"))

(in-package "ROS")
;;//! \htmlinclude Base_RotationMatrixRow.msg.html


(defclass kortex_driver::Base_RotationMatrixRow
  :super ros::object
  :slots (_column1 _column2 _column3 ))

(defmethod kortex_driver::Base_RotationMatrixRow
  (:init
   (&key
    ((:column1 __column1) 0.0)
    ((:column2 __column2) 0.0)
    ((:column3 __column3) 0.0)
    )
   (send-super :init)
   (setq _column1 (float __column1))
   (setq _column2 (float __column2))
   (setq _column3 (float __column3))
   self)
  (:column1
   (&optional __column1)
   (if __column1 (setq _column1 __column1)) _column1)
  (:column2
   (&optional __column2)
   (if __column2 (setq _column2 __column2)) _column2)
  (:column3
   (&optional __column3)
   (if __column3 (setq _column3 __column3)) _column3)
  (:serialization-length
   ()
   (+
    ;; float32 _column1
    4
    ;; float32 _column2
    4
    ;; float32 _column3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _column1
       (sys::poke _column1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _column2
       (sys::poke _column2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _column3
       (sys::poke _column3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _column1
     (setq _column1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _column2
     (setq _column2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _column3
     (setq _column3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Base_RotationMatrixRow :md5sum-) "7749fbbc183a47c4c30d1e4e586c8b66")
(setf (get kortex_driver::Base_RotationMatrixRow :datatype-) "kortex_driver/Base_RotationMatrixRow")
(setf (get kortex_driver::Base_RotationMatrixRow :definition-)
      "
float32 column1
float32 column2
float32 column3
")



(provide :kortex_driver/Base_RotationMatrixRow "7749fbbc183a47c4c30d1e4e586c8b66")


