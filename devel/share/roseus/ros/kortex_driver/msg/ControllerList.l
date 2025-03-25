;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERLIST")
  (make-package "KORTEX_DRIVER::CONTROLLERLIST"))

(in-package "ROS")
;;//! \htmlinclude ControllerList.msg.html


(defclass kortex_driver::ControllerList
  :super ros::object
  :slots (_handles ))

(defmethod kortex_driver::ControllerList
  (:init
   (&key
    ((:handles __handles) ())
    )
   (send-super :init)
   (setq _handles __handles)
   self)
  (:handles
   (&rest __handles)
   (if (keywordp (car __handles))
       (send* _handles __handles)
     (progn
       (if __handles (setq _handles (car __handles)))
       _handles)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerHandle[] _handles
    (apply #'+ (send-all _handles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle[] _handles
     (write-long (length _handles) s)
     (dolist (elem _handles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle[] _handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _handles (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerHandle :init) r)) r))
     (dolist (elem- _handles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerList :md5sum-) "952a49d983a336790fc984bb01df940e")
(setf (get kortex_driver::ControllerList :datatype-) "kortex_driver/ControllerList")
(setf (get kortex_driver::ControllerList :definition-)
      "
ControllerHandle[] handles
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
")



(provide :kortex_driver/ControllerList "952a49d983a336790fc984bb01df940e")


