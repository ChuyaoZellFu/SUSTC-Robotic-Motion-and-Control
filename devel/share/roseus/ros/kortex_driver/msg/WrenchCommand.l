;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WrenchCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WrenchCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WRENCHCOMMAND")
  (make-package "KORTEX_DRIVER::WRENCHCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude WrenchCommand.msg.html


(defclass kortex_driver::WrenchCommand
  :super ros::object
  :slots (_reference_frame _mode _wrench _duration ))

(defmethod kortex_driver::WrenchCommand
  (:init
   (&key
    ((:reference_frame __reference_frame) 0)
    ((:mode __mode) 0)
    ((:wrench __wrench) (instance kortex_driver::Wrench :init))
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _reference_frame (round __reference_frame))
   (setq _mode (round __mode))
   (setq _wrench __wrench)
   (setq _duration (round __duration))
   self)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _reference_frame
    4
    ;; uint32 _mode
    4
    ;; kortex_driver/Wrench _wrench
    (send _wrench :serialization-length)
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _reference_frame
       (write-long _reference_frame s)
     ;; uint32 _mode
       (write-long _mode s)
     ;; kortex_driver/Wrench _wrench
       (send _wrench :serialize s)
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _reference_frame
     (setq _reference_frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::WrenchCommand :md5sum-) "76c8b85ff99edd42232a407e2d24c818")
(setf (get kortex_driver::WrenchCommand :datatype-) "kortex_driver/WrenchCommand")
(setf (get kortex_driver::WrenchCommand :definition-)
      "
uint32 reference_frame
uint32 mode
Wrench wrench
uint32 duration
================================================================================
MSG: kortex_driver/Wrench

float32 force_x
float32 force_y
float32 force_z
float32 torque_x
float32 torque_y
float32 torque_z
")



(provide :kortex_driver/WrenchCommand "76c8b85ff99edd42232a407e2d24c818")


