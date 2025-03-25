;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_RotationMatrix)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_RotationMatrix (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_ROTATIONMATRIX")
  (make-package "KORTEX_DRIVER::BASE_ROTATIONMATRIX"))

(in-package "ROS")
;;//! \htmlinclude Base_RotationMatrix.msg.html


(defclass kortex_driver::Base_RotationMatrix
  :super ros::object
  :slots (_row1 _row2 _row3 ))

(defmethod kortex_driver::Base_RotationMatrix
  (:init
   (&key
    ((:row1 __row1) (instance kortex_driver::Base_RotationMatrixRow :init))
    ((:row2 __row2) (instance kortex_driver::Base_RotationMatrixRow :init))
    ((:row3 __row3) (instance kortex_driver::Base_RotationMatrixRow :init))
    )
   (send-super :init)
   (setq _row1 __row1)
   (setq _row2 __row2)
   (setq _row3 __row3)
   self)
  (:row1
   (&rest __row1)
   (if (keywordp (car __row1))
       (send* _row1 __row1)
     (progn
       (if __row1 (setq _row1 (car __row1)))
       _row1)))
  (:row2
   (&rest __row2)
   (if (keywordp (car __row2))
       (send* _row2 __row2)
     (progn
       (if __row2 (setq _row2 (car __row2)))
       _row2)))
  (:row3
   (&rest __row3)
   (if (keywordp (car __row3))
       (send* _row3 __row3)
     (progn
       (if __row3 (setq _row3 (car __row3)))
       _row3)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Base_RotationMatrixRow _row1
    (send _row1 :serialization-length)
    ;; kortex_driver/Base_RotationMatrixRow _row2
    (send _row2 :serialization-length)
    ;; kortex_driver/Base_RotationMatrixRow _row3
    (send _row3 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_RotationMatrixRow _row1
       (send _row1 :serialize s)
     ;; kortex_driver/Base_RotationMatrixRow _row2
       (send _row2 :serialize s)
     ;; kortex_driver/Base_RotationMatrixRow _row3
       (send _row3 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_RotationMatrixRow _row1
     (send _row1 :deserialize buf ptr-) (incf ptr- (send _row1 :serialization-length))
   ;; kortex_driver/Base_RotationMatrixRow _row2
     (send _row2 :deserialize buf ptr-) (incf ptr- (send _row2 :serialization-length))
   ;; kortex_driver/Base_RotationMatrixRow _row3
     (send _row3 :deserialize buf ptr-) (incf ptr- (send _row3 :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::Base_RotationMatrix :md5sum-) "37bab3875ca8eda43ce39de02fa3b72b")
(setf (get kortex_driver::Base_RotationMatrix :datatype-) "kortex_driver/Base_RotationMatrix")
(setf (get kortex_driver::Base_RotationMatrix :definition-)
      "
Base_RotationMatrixRow row1
Base_RotationMatrixRow row2
Base_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/Base_RotationMatrixRow

float32 column1
float32 column2
float32 column3
")



(provide :kortex_driver/Base_RotationMatrix "37bab3875ca8eda43ce39de02fa3b72b")


