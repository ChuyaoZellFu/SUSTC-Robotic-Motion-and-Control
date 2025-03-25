;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StartWifiScan)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StartWifiScan (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STARTWIFISCAN")
  (make-package "KORTEX_DRIVER::STARTWIFISCAN"))
(unless (find-package "KORTEX_DRIVER::STARTWIFISCANREQUEST")
  (make-package "KORTEX_DRIVER::STARTWIFISCANREQUEST"))
(unless (find-package "KORTEX_DRIVER::STARTWIFISCANRESPONSE")
  (make-package "KORTEX_DRIVER::STARTWIFISCANRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StartWifiScanRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StartWifiScanRequest
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

(defclass kortex_driver::StartWifiScanResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StartWifiScanResponse
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

(defclass kortex_driver::StartWifiScan
  :super ros::object
  :slots ())

(setf (get kortex_driver::StartWifiScan :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StartWifiScan :datatype-) "kortex_driver/StartWifiScan")
(setf (get kortex_driver::StartWifiScan :request) kortex_driver::StartWifiScanRequest)
(setf (get kortex_driver::StartWifiScan :response) kortex_driver::StartWifiScanResponse)

(defmethod kortex_driver::StartWifiScanRequest
  (:response () (instance kortex_driver::StartWifiScanResponse :init)))

(setf (get kortex_driver::StartWifiScanRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StartWifiScanRequest :datatype-) "kortex_driver/StartWifiScanRequest")
(setf (get kortex_driver::StartWifiScanRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::StartWifiScanResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StartWifiScanResponse :datatype-) "kortex_driver/StartWifiScanResponse")
(setf (get kortex_driver::StartWifiScanResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/StartWifiScan "f335b819dc59099fe3124b36f140ad07")


