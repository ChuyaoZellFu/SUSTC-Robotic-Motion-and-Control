;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TwistCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TwistCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TWISTCOMMAND")
  (make-package "KORTEX_DRIVER::TWISTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude TwistCommand.msg.html


(defclass kortex_driver::TwistCommand
  :super ros::object
  :slots (_reference_frame _twist _duration ))

(defmethod kortex_driver::TwistCommand
  (:init
   (&key
    ((:reference_frame __reference_frame) 0)
    ((:twist __twist) (instance kortex_driver::Twist :init))
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _reference_frame (round __reference_frame))
   (setq _twist __twist)
   (setq _duration (round __duration))
   self)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _reference_frame
    4
    ;; kortex_driver/Twist _twist
    (send _twist :serialization-length)
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _reference_frame
       (write-long _reference_frame s)
     ;; kortex_driver/Twist _twist
       (send _twist :serialize s)
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _reference_frame
     (setq _reference_frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TwistCommand :md5sum-) "16dcfd20a022a10eea1f05e5a9cbb18a")
(setf (get kortex_driver::TwistCommand :datatype-) "kortex_driver/TwistCommand")
(setf (get kortex_driver::TwistCommand :definition-)
      "
uint32 reference_frame
Twist twist
uint32 duration
================================================================================
MSG: kortex_driver/Twist

float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
float32 angular_z
")



(provide :kortex_driver/TwistCommand "16dcfd20a022a10eea1f05e5a9cbb18a")


