;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CoggingFeedforwardModeInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CoggingFeedforwardModeInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODEINFORMATION")
  (make-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude CoggingFeedforwardModeInformation.msg.html


(defclass kortex_driver::CoggingFeedforwardModeInformation
  :super ros::object
  :slots (_cogging_feedforward_mode ))

(defmethod kortex_driver::CoggingFeedforwardModeInformation
  (:init
   (&key
    ((:cogging_feedforward_mode __cogging_feedforward_mode) 0)
    )
   (send-super :init)
   (setq _cogging_feedforward_mode (round __cogging_feedforward_mode))
   self)
  (:cogging_feedforward_mode
   (&optional __cogging_feedforward_mode)
   (if __cogging_feedforward_mode (setq _cogging_feedforward_mode __cogging_feedforward_mode)) _cogging_feedforward_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _cogging_feedforward_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _cogging_feedforward_mode
       (write-long _cogging_feedforward_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _cogging_feedforward_mode
     (setq _cogging_feedforward_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CoggingFeedforwardModeInformation :md5sum-) "e26fc90c6a8fc2a4d783f21f241b9871")
(setf (get kortex_driver::CoggingFeedforwardModeInformation :datatype-) "kortex_driver/CoggingFeedforwardModeInformation")
(setf (get kortex_driver::CoggingFeedforwardModeInformation :definition-)
      "
uint32 cogging_feedforward_mode
")



(provide :kortex_driver/CoggingFeedforwardModeInformation "e26fc90c6a8fc2a4d783f21f241b9871")


