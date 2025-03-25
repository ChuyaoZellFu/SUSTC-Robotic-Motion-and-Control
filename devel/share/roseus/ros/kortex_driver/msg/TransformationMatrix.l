;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TransformationMatrix)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TransformationMatrix (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRANSFORMATIONMATRIX")
  (make-package "KORTEX_DRIVER::TRANSFORMATIONMATRIX"))

(in-package "ROS")
;;//! \htmlinclude TransformationMatrix.msg.html


(defclass kortex_driver::TransformationMatrix
  :super ros::object
  :slots (_r0 _r1 _r2 _r3 ))

(defmethod kortex_driver::TransformationMatrix
  (:init
   (&key
    ((:r0 __r0) (instance kortex_driver::TransformationRow :init))
    ((:r1 __r1) (instance kortex_driver::TransformationRow :init))
    ((:r2 __r2) (instance kortex_driver::TransformationRow :init))
    ((:r3 __r3) (instance kortex_driver::TransformationRow :init))
    )
   (send-super :init)
   (setq _r0 __r0)
   (setq _r1 __r1)
   (setq _r2 __r2)
   (setq _r3 __r3)
   self)
  (:r0
   (&rest __r0)
   (if (keywordp (car __r0))
       (send* _r0 __r0)
     (progn
       (if __r0 (setq _r0 (car __r0)))
       _r0)))
  (:r1
   (&rest __r1)
   (if (keywordp (car __r1))
       (send* _r1 __r1)
     (progn
       (if __r1 (setq _r1 (car __r1)))
       _r1)))
  (:r2
   (&rest __r2)
   (if (keywordp (car __r2))
       (send* _r2 __r2)
     (progn
       (if __r2 (setq _r2 (car __r2)))
       _r2)))
  (:r3
   (&rest __r3)
   (if (keywordp (car __r3))
       (send* _r3 __r3)
     (progn
       (if __r3 (setq _r3 (car __r3)))
       _r3)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/TransformationRow _r0
    (send _r0 :serialization-length)
    ;; kortex_driver/TransformationRow _r1
    (send _r1 :serialization-length)
    ;; kortex_driver/TransformationRow _r2
    (send _r2 :serialization-length)
    ;; kortex_driver/TransformationRow _r3
    (send _r3 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TransformationRow _r0
       (send _r0 :serialize s)
     ;; kortex_driver/TransformationRow _r1
       (send _r1 :serialize s)
     ;; kortex_driver/TransformationRow _r2
       (send _r2 :serialize s)
     ;; kortex_driver/TransformationRow _r3
       (send _r3 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TransformationRow _r0
     (send _r0 :deserialize buf ptr-) (incf ptr- (send _r0 :serialization-length))
   ;; kortex_driver/TransformationRow _r1
     (send _r1 :deserialize buf ptr-) (incf ptr- (send _r1 :serialization-length))
   ;; kortex_driver/TransformationRow _r2
     (send _r2 :deserialize buf ptr-) (incf ptr- (send _r2 :serialization-length))
   ;; kortex_driver/TransformationRow _r3
     (send _r3 :deserialize buf ptr-) (incf ptr- (send _r3 :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::TransformationMatrix :md5sum-) "01b579348ba31feab01058d980795bad")
(setf (get kortex_driver::TransformationMatrix :datatype-) "kortex_driver/TransformationMatrix")
(setf (get kortex_driver::TransformationMatrix :definition-)
      "
TransformationRow r0
TransformationRow r1
TransformationRow r2
TransformationRow r3
================================================================================
MSG: kortex_driver/TransformationRow

float32 c0
float32 c1
float32 c2
float32 c3
")



(provide :kortex_driver/TransformationMatrix "01b579348ba31feab01058d980795bad")


