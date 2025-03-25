;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERCONFIGURATION")
  (make-package "KORTEX_DRIVER::CONTROLLERCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude ControllerConfiguration.msg.html


(defclass kortex_driver::ControllerConfiguration
  :super ros::object
  :slots (_handle _name _active_mapping_handle _analog_input_identifier_enum _digital_input_identifier_enum ))

(defmethod kortex_driver::ControllerConfiguration
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::ControllerHandle :init))
    ((:name __name) "")
    ((:active_mapping_handle __active_mapping_handle) (instance kortex_driver::MappingHandle :init))
    ((:analog_input_identifier_enum __analog_input_identifier_enum) "")
    ((:digital_input_identifier_enum __digital_input_identifier_enum) "")
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _name (string __name))
   (setq _active_mapping_handle __active_mapping_handle)
   (setq _analog_input_identifier_enum (string __analog_input_identifier_enum))
   (setq _digital_input_identifier_enum (string __digital_input_identifier_enum))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:active_mapping_handle
   (&rest __active_mapping_handle)
   (if (keywordp (car __active_mapping_handle))
       (send* _active_mapping_handle __active_mapping_handle)
     (progn
       (if __active_mapping_handle (setq _active_mapping_handle (car __active_mapping_handle)))
       _active_mapping_handle)))
  (:analog_input_identifier_enum
   (&optional __analog_input_identifier_enum)
   (if __analog_input_identifier_enum (setq _analog_input_identifier_enum __analog_input_identifier_enum)) _analog_input_identifier_enum)
  (:digital_input_identifier_enum
   (&optional __digital_input_identifier_enum)
   (if __digital_input_identifier_enum (setq _digital_input_identifier_enum __digital_input_identifier_enum)) _digital_input_identifier_enum)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerHandle _handle
    (send _handle :serialization-length)
    ;; string _name
    4 (length _name)
    ;; kortex_driver/MappingHandle _active_mapping_handle
    (send _active_mapping_handle :serialization-length)
    ;; string _analog_input_identifier_enum
    4 (length _analog_input_identifier_enum)
    ;; string _digital_input_identifier_enum
    4 (length _digital_input_identifier_enum)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle _handle
       (send _handle :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; kortex_driver/MappingHandle _active_mapping_handle
       (send _active_mapping_handle :serialize s)
     ;; string _analog_input_identifier_enum
       (write-long (length _analog_input_identifier_enum) s) (princ _analog_input_identifier_enum s)
     ;; string _digital_input_identifier_enum
       (write-long (length _digital_input_identifier_enum) s) (princ _digital_input_identifier_enum s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/MappingHandle _active_mapping_handle
     (send _active_mapping_handle :deserialize buf ptr-) (incf ptr- (send _active_mapping_handle :serialization-length))
   ;; string _analog_input_identifier_enum
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _analog_input_identifier_enum (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _digital_input_identifier_enum
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _digital_input_identifier_enum (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerConfiguration :md5sum-) "0a2e41d50fc9c491b9a0c9000a90ca85")
(setf (get kortex_driver::ControllerConfiguration :datatype-) "kortex_driver/ControllerConfiguration")
(setf (get kortex_driver::ControllerConfiguration :definition-)
      "
ControllerHandle handle
string name
MappingHandle active_mapping_handle
string analog_input_identifier_enum
string digital_input_identifier_enum
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/ControllerConfiguration "0a2e41d50fc9c491b9a0c9000a90ca85")


