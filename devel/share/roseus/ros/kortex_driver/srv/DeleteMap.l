;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteMap)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteMap (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEMAP")
  (make-package "KORTEX_DRIVER::DELETEMAP"))
(unless (find-package "KORTEX_DRIVER::DELETEMAPREQUEST")
  (make-package "KORTEX_DRIVER::DELETEMAPREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEMAPRESPONSE")
  (make-package "KORTEX_DRIVER::DELETEMAPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteMapRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteMapRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::MapHandle :init))
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
    ;; kortex_driver/MapHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MapHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MapHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeleteMapResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteMapResponse
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

(defclass kortex_driver::DeleteMap
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteMap :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMap :datatype-) "kortex_driver/DeleteMap")
(setf (get kortex_driver::DeleteMap :request) kortex_driver::DeleteMapRequest)
(setf (get kortex_driver::DeleteMap :response) kortex_driver::DeleteMapResponse)

(defmethod kortex_driver::DeleteMapRequest
  (:response () (instance kortex_driver::DeleteMapResponse :init)))

(setf (get kortex_driver::DeleteMapRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMapRequest :datatype-) "kortex_driver/DeleteMapRequest")
(setf (get kortex_driver::DeleteMapRequest :definition-)
      "MapHandle input

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteMapResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMapResponse :datatype-) "kortex_driver/DeleteMapResponse")
(setf (get kortex_driver::DeleteMapResponse :definition-)
      "MapHandle input

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteMap "04dfaeca45772f660e0913aa84774e13")


