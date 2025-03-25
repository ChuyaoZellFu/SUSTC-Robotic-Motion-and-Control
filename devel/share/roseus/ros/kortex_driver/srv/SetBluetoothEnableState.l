;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetBluetoothEnableState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetBluetoothEnableState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATE")
  (make-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATE"))
(unless (find-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATEREQUEST")
  (make-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATERESPONSE")
  (make-package "KORTEX_DRIVER::SETBLUETOOTHENABLESTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetBluetoothEnableStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetBluetoothEnableStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::BluetoothEnableState :init))
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
    ;; kortex_driver/BluetoothEnableState _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BluetoothEnableState _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BluetoothEnableState _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetBluetoothEnableStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetBluetoothEnableStateResponse
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

(defclass kortex_driver::SetBluetoothEnableState
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetBluetoothEnableState :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetBluetoothEnableState :datatype-) "kortex_driver/SetBluetoothEnableState")
(setf (get kortex_driver::SetBluetoothEnableState :request) kortex_driver::SetBluetoothEnableStateRequest)
(setf (get kortex_driver::SetBluetoothEnableState :response) kortex_driver::SetBluetoothEnableStateResponse)

(defmethod kortex_driver::SetBluetoothEnableStateRequest
  (:response () (instance kortex_driver::SetBluetoothEnableStateResponse :init)))

(setf (get kortex_driver::SetBluetoothEnableStateRequest :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetBluetoothEnableStateRequest :datatype-) "kortex_driver/SetBluetoothEnableStateRequest")
(setf (get kortex_driver::SetBluetoothEnableStateRequest :definition-)
      "BluetoothEnableState input

================================================================================
MSG: kortex_driver/BluetoothEnableState

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetBluetoothEnableStateResponse :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetBluetoothEnableStateResponse :datatype-) "kortex_driver/SetBluetoothEnableStateResponse")
(setf (get kortex_driver::SetBluetoothEnableStateResponse :definition-)
      "BluetoothEnableState input

================================================================================
MSG: kortex_driver/BluetoothEnableState

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetBluetoothEnableState "7ccd6a6c56b1642433541444d0e2ab2f")


