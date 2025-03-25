;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Gripper)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Gripper (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPER")
  (make-package "KORTEX_DRIVER::GRIPPER"))

(in-package "ROS")
;;//! \htmlinclude Gripper.msg.html


(defclass kortex_driver::Gripper
  :super ros::object
  :slots (_finger ))

(defmethod kortex_driver::Gripper
  (:init
   (&key
    ((:finger __finger) ())
    )
   (send-super :init)
   (setq _finger __finger)
   self)
  (:finger
   (&rest __finger)
   (if (keywordp (car __finger))
       (send* _finger __finger)
     (progn
       (if __finger (setq _finger (car __finger)))
       _finger)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Finger[] _finger
    (apply #'+ (send-all _finger :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Finger[] _finger
     (write-long (length _finger) s)
     (dolist (elem _finger)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Finger[] _finger
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _finger (let (r) (dotimes (i n) (push (instance kortex_driver::Finger :init) r)) r))
     (dolist (elem- _finger)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::Gripper :md5sum-) "d6de7f2f0e25d1f1553a193c7d388b15")
(setf (get kortex_driver::Gripper :datatype-) "kortex_driver/Gripper")
(setf (get kortex_driver::Gripper :definition-)
      "
Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
float32 value
")



(provide :kortex_driver/Gripper "d6de7f2f0e25d1f1553a193c7d388b15")


