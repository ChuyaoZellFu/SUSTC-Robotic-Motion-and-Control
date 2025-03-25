;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointsLimitationsList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointsLimitationsList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTSLIMITATIONSLIST")
  (make-package "KORTEX_DRIVER::JOINTSLIMITATIONSLIST"))

(in-package "ROS")
;;//! \htmlinclude JointsLimitationsList.msg.html


(defclass kortex_driver::JointsLimitationsList
  :super ros::object
  :slots (_joints_limitations ))

(defmethod kortex_driver::JointsLimitationsList
  (:init
   (&key
    ((:joints_limitations __joints_limitations) ())
    )
   (send-super :init)
   (setq _joints_limitations __joints_limitations)
   self)
  (:joints_limitations
   (&rest __joints_limitations)
   (if (keywordp (car __joints_limitations))
       (send* _joints_limitations __joints_limitations)
     (progn
       (if __joints_limitations (setq _joints_limitations (car __joints_limitations)))
       _joints_limitations)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/JointLimitation[] _joints_limitations
    (apply #'+ (send-all _joints_limitations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointLimitation[] _joints_limitations
     (write-long (length _joints_limitations) s)
     (dolist (elem _joints_limitations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointLimitation[] _joints_limitations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joints_limitations (let (r) (dotimes (i n) (push (instance kortex_driver::JointLimitation :init) r)) r))
     (dolist (elem- _joints_limitations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::JointsLimitationsList :md5sum-) "6dfe2c39e44634a2eba8f7879d518184")
(setf (get kortex_driver::JointsLimitationsList :datatype-) "kortex_driver/JointsLimitationsList")
(setf (get kortex_driver::JointsLimitationsList :definition-)
      "
JointLimitation[] joints_limitations
================================================================================
MSG: kortex_driver/JointLimitation

uint32 joint_identifier
uint32 type
float32 value
")



(provide :kortex_driver/JointsLimitationsList "6dfe2c39e44634a2eba8f7879d518184")


