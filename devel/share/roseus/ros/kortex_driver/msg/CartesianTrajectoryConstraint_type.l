;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianTrajectoryConstraint_type)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianTrajectoryConstraint_type (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANTRAJECTORYCONSTRAINT_TYPE")
  (make-package "KORTEX_DRIVER::CARTESIANTRAJECTORYCONSTRAINT_TYPE"))

(in-package "ROS")
;;//! \htmlinclude CartesianTrajectoryConstraint_type.msg.html


(defclass kortex_driver::CartesianTrajectoryConstraint_type
  :super ros::object
  :slots (_speed _duration ))

(defmethod kortex_driver::CartesianTrajectoryConstraint_type
  (:init
   (&key
    ((:speed __speed) ())
    ((:duration __duration) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _speed __speed)
   (setq _duration __duration)
   self)
  (:speed
   (&rest __speed)
   (if (keywordp (car __speed))
       (send* _speed __speed)
     (progn
       (if __speed (setq _speed (car __speed)))
       _speed)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/CartesianSpeed[] _speed
    (apply #'+ (send-all _speed :serialization-length)) 4
    ;; uint32[] _duration
    (* 4    (length _duration)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianSpeed[] _speed
     (write-long (length _speed) s)
     (dolist (elem _speed)
       (send elem :serialize s)
       )
     ;; uint32[] _duration
     (write-long (length _duration) s)
     (dotimes (i (length _duration))
       (write-long (elt _duration i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianSpeed[] _speed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speed (let (r) (dotimes (i n) (push (instance kortex_driver::CartesianSpeed :init) r)) r))
     (dolist (elem- _speed)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32[] _duration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _duration (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _duration i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::CartesianTrajectoryConstraint_type :md5sum-) "e14536e767e1f700827f285e4e6fd832")
(setf (get kortex_driver::CartesianTrajectoryConstraint_type :datatype-) "kortex_driver/CartesianTrajectoryConstraint_type")
(setf (get kortex_driver::CartesianTrajectoryConstraint_type :definition-)
      "
CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
float32 orientation
")



(provide :kortex_driver/CartesianTrajectoryConstraint_type "e14536e767e1f700827f285e4e6fd832")


