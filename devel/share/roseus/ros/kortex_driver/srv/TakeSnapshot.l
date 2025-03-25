;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TakeSnapshot)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TakeSnapshot (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TAKESNAPSHOT")
  (make-package "KORTEX_DRIVER::TAKESNAPSHOT"))
(unless (find-package "KORTEX_DRIVER::TAKESNAPSHOTREQUEST")
  (make-package "KORTEX_DRIVER::TAKESNAPSHOTREQUEST"))
(unless (find-package "KORTEX_DRIVER::TAKESNAPSHOTRESPONSE")
  (make-package "KORTEX_DRIVER::TAKESNAPSHOTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::TakeSnapshotRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::TakeSnapshotRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Snapshot :init))
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
    ;; kortex_driver/Snapshot _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Snapshot _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Snapshot _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::TakeSnapshotResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::TakeSnapshotResponse
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

(defclass kortex_driver::TakeSnapshot
  :super ros::object
  :slots ())

(setf (get kortex_driver::TakeSnapshot :md5sum-) "4fa3de1c55135bce6fbd25c77b5e21af")
(setf (get kortex_driver::TakeSnapshot :datatype-) "kortex_driver/TakeSnapshot")
(setf (get kortex_driver::TakeSnapshot :request) kortex_driver::TakeSnapshotRequest)
(setf (get kortex_driver::TakeSnapshot :response) kortex_driver::TakeSnapshotResponse)

(defmethod kortex_driver::TakeSnapshotRequest
  (:response () (instance kortex_driver::TakeSnapshotResponse :init)))

(setf (get kortex_driver::TakeSnapshotRequest :md5sum-) "4fa3de1c55135bce6fbd25c77b5e21af")
(setf (get kortex_driver::TakeSnapshotRequest :datatype-) "kortex_driver/TakeSnapshotRequest")
(setf (get kortex_driver::TakeSnapshotRequest :definition-)
      "Snapshot input

================================================================================
MSG: kortex_driver/Snapshot

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::TakeSnapshotResponse :md5sum-) "4fa3de1c55135bce6fbd25c77b5e21af")
(setf (get kortex_driver::TakeSnapshotResponse :datatype-) "kortex_driver/TakeSnapshotResponse")
(setf (get kortex_driver::TakeSnapshotResponse :definition-)
      "Snapshot input

================================================================================
MSG: kortex_driver/Snapshot

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/TakeSnapshot "4fa3de1c55135bce6fbd25c77b5e21af")


