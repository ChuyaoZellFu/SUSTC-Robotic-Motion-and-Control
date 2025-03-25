;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SensorFocusAction_action_parameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SensorFocusAction_action_parameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENSORFOCUSACTION_ACTION_PARAMETERS")
  (make-package "KORTEX_DRIVER::SENSORFOCUSACTION_ACTION_PARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude SensorFocusAction_action_parameters.msg.html


(defclass kortex_driver::SensorFocusAction_action_parameters
  :super ros::object
  :slots (_focus_point _manual_focus ))

(defmethod kortex_driver::SensorFocusAction_action_parameters
  (:init
   (&key
    ((:focus_point __focus_point) ())
    ((:manual_focus __manual_focus) ())
    )
   (send-super :init)
   (setq _focus_point __focus_point)
   (setq _manual_focus __manual_focus)
   self)
  (:focus_point
   (&rest __focus_point)
   (if (keywordp (car __focus_point))
       (send* _focus_point __focus_point)
     (progn
       (if __focus_point (setq _focus_point (car __focus_point)))
       _focus_point)))
  (:manual_focus
   (&rest __manual_focus)
   (if (keywordp (car __manual_focus))
       (send* _manual_focus __manual_focus)
     (progn
       (if __manual_focus (setq _manual_focus (car __manual_focus)))
       _manual_focus)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/FocusPoint[] _focus_point
    (apply #'+ (send-all _focus_point :serialization-length)) 4
    ;; kortex_driver/ManualFocus[] _manual_focus
    (apply #'+ (send-all _manual_focus :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/FocusPoint[] _focus_point
     (write-long (length _focus_point) s)
     (dolist (elem _focus_point)
       (send elem :serialize s)
       )
     ;; kortex_driver/ManualFocus[] _manual_focus
     (write-long (length _manual_focus) s)
     (dolist (elem _manual_focus)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/FocusPoint[] _focus_point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _focus_point (let (r) (dotimes (i n) (push (instance kortex_driver::FocusPoint :init) r)) r))
     (dolist (elem- _focus_point)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ManualFocus[] _manual_focus
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _manual_focus (let (r) (dotimes (i n) (push (instance kortex_driver::ManualFocus :init) r)) r))
     (dolist (elem- _manual_focus)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::SensorFocusAction_action_parameters :md5sum-) "6e07eb3126a69325e87d9841916b8dc3")
(setf (get kortex_driver::SensorFocusAction_action_parameters :datatype-) "kortex_driver/SensorFocusAction_action_parameters")
(setf (get kortex_driver::SensorFocusAction_action_parameters :definition-)
      "
FocusPoint[] focus_point
ManualFocus[] manual_focus
================================================================================
MSG: kortex_driver/FocusPoint

uint32 x
uint32 y
================================================================================
MSG: kortex_driver/ManualFocus

uint32 value
")



(provide :kortex_driver/SensorFocusAction_action_parameters "6e07eb3126a69325e87d9841916b8dc3")


