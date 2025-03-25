;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FocusAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FocusAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FOCUSACTION")
  (make-package "KORTEX_DRIVER::FOCUSACTION"))

(in-package "ROS")
;;//! \htmlinclude FocusAction.msg.html


(intern "*FOCUSACTION_UNSPECIFIED*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_UNSPECIFIED* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_UNSPECIFIED* 0)
(intern "*FOCUSACTION_START_CONTINUOUS_FOCUS*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_START_CONTINUOUS_FOCUS* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_START_CONTINUOUS_FOCUS* 1)
(intern "*FOCUSACTION_PAUSE_CONTINUOUS_FOCUS*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_PAUSE_CONTINUOUS_FOCUS* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_PAUSE_CONTINUOUS_FOCUS* 2)
(intern "*FOCUSACTION_FOCUS_NOW*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_FOCUS_NOW* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_FOCUS_NOW* 3)
(intern "*FOCUSACTION_DISABLE_FOCUS*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_DISABLE_FOCUS* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_DISABLE_FOCUS* 4)
(intern "*FOCUSACTION_SET_FOCUS_POINT*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_SET_FOCUS_POINT* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_SET_FOCUS_POINT* 5)
(intern "*FOCUSACTION_SET_MANUAL_FOCUS*" (find-package "KORTEX_DRIVER::FOCUSACTION"))
(shadow '*FOCUSACTION_SET_MANUAL_FOCUS* (find-package "KORTEX_DRIVER::FOCUSACTION"))
(defconstant kortex_driver::FocusAction::*FOCUSACTION_SET_MANUAL_FOCUS* 6)

(defun kortex_driver::FocusAction-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::FocusAction::*FOCUSACTION_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::FocusAction::*FOCUSACTION_START_CONTINUOUS_FOCUS*)
        ((= const 2) 'kortex_driver::FocusAction::*FOCUSACTION_PAUSE_CONTINUOUS_FOCUS*)
        ((= const 3) 'kortex_driver::FocusAction::*FOCUSACTION_FOCUS_NOW*)
        ((= const 4) 'kortex_driver::FocusAction::*FOCUSACTION_DISABLE_FOCUS*)
        ((= const 5) 'kortex_driver::FocusAction::*FOCUSACTION_SET_FOCUS_POINT*)
        ((= const 6) 'kortex_driver::FocusAction::*FOCUSACTION_SET_MANUAL_FOCUS*)
        (t nil)))

(defclass kortex_driver::FocusAction
  :super ros::object
  :slots ())

(defmethod kortex_driver::FocusAction
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::FocusAction :md5sum-) "16232e452a5de2238b9046f807acd748")
(setf (get kortex_driver::FocusAction :datatype-) "kortex_driver/FocusAction")
(setf (get kortex_driver::FocusAction :definition-)
      "
uint32 FOCUSACTION_UNSPECIFIED = 0

uint32 FOCUSACTION_START_CONTINUOUS_FOCUS = 1

uint32 FOCUSACTION_PAUSE_CONTINUOUS_FOCUS = 2

uint32 FOCUSACTION_FOCUS_NOW = 3

uint32 FOCUSACTION_DISABLE_FOCUS = 4

uint32 FOCUSACTION_SET_FOCUS_POINT = 5

uint32 FOCUSACTION_SET_MANUAL_FOCUS = 6

")



(provide :kortex_driver/FocusAction "16232e452a5de2238b9046f807acd748")


