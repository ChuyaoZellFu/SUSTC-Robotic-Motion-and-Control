;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DisconnectWifi)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DisconnectWifi (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DISCONNECTWIFI")
  (make-package "KORTEX_DRIVER::DISCONNECTWIFI"))
(unless (find-package "KORTEX_DRIVER::DISCONNECTWIFIREQUEST")
  (make-package "KORTEX_DRIVER::DISCONNECTWIFIREQUEST"))
(unless (find-package "KORTEX_DRIVER::DISCONNECTWIFIRESPONSE")
  (make-package "KORTEX_DRIVER::DISCONNECTWIFIRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DisconnectWifiRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DisconnectWifiRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DisconnectWifiResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DisconnectWifiResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DisconnectWifi
  :super ros::object
  :slots ())

(setf (get kortex_driver::DisconnectWifi :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::DisconnectWifi :datatype-) "kortex_driver/DisconnectWifi")
(setf (get kortex_driver::DisconnectWifi :request) kortex_driver::DisconnectWifiRequest)
(setf (get kortex_driver::DisconnectWifi :response) kortex_driver::DisconnectWifiResponse)

(defmethod kortex_driver::DisconnectWifiRequest
  (:response () (instance kortex_driver::DisconnectWifiResponse :init)))

(setf (get kortex_driver::DisconnectWifiRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::DisconnectWifiRequest :datatype-) "kortex_driver/DisconnectWifiRequest")
(setf (get kortex_driver::DisconnectWifiRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DisconnectWifiResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::DisconnectWifiResponse :datatype-) "kortex_driver/DisconnectWifiResponse")
(setf (get kortex_driver::DisconnectWifiResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DisconnectWifi "f335b819dc59099fe3124b36f140ad07")


