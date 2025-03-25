;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetBluetoothEnableState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetBluetoothEnableState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATE")
  (make-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATE"))
(unless (find-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATEREQUEST")
  (make-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATERESPONSE")
  (make-package "KORTEX_DRIVER::GETBLUETOOTHENABLESTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetBluetoothEnableStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetBluetoothEnableStateRequest
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

(defclass kortex_driver::GetBluetoothEnableStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetBluetoothEnableStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::BluetoothEnableState :init))
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
    ;; kortex_driver/BluetoothEnableState _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BluetoothEnableState _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BluetoothEnableState _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBluetoothEnableState
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetBluetoothEnableState :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetBluetoothEnableState :datatype-) "kortex_driver/GetBluetoothEnableState")
(setf (get kortex_driver::GetBluetoothEnableState :request) kortex_driver::GetBluetoothEnableStateRequest)
(setf (get kortex_driver::GetBluetoothEnableState :response) kortex_driver::GetBluetoothEnableStateResponse)

(defmethod kortex_driver::GetBluetoothEnableStateRequest
  (:response () (instance kortex_driver::GetBluetoothEnableStateResponse :init)))

(setf (get kortex_driver::GetBluetoothEnableStateRequest :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetBluetoothEnableStateRequest :datatype-) "kortex_driver/GetBluetoothEnableStateRequest")
(setf (get kortex_driver::GetBluetoothEnableStateRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BluetoothEnableState output

================================================================================
MSG: kortex_driver/BluetoothEnableState

")

(setf (get kortex_driver::GetBluetoothEnableStateResponse :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetBluetoothEnableStateResponse :datatype-) "kortex_driver/GetBluetoothEnableStateResponse")
(setf (get kortex_driver::GetBluetoothEnableStateResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BluetoothEnableState output

================================================================================
MSG: kortex_driver/BluetoothEnableState

")



(provide :kortex_driver/GetBluetoothEnableState "9747040002a13b23ba7503e4b2f380fb")


