; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionNotification.msg.html

(cl:defclass <VisionNotification> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:integer
    :initform 0)
   (sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (option
    :reader option
    :initarg :option
    :type cl:integer
    :initform 0))
)

(cl:defclass VisionNotification (<VisionNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionNotification> is deprecated: use kortex_driver-msg:VisionNotification instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <VisionNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event-val is deprecated.  Use kortex_driver-msg:event instead.")
  (event m))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <VisionNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'option-val :lambda-list '(m))
(cl:defmethod option-val ((m <VisionNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:option-val is deprecated.  Use kortex_driver-msg:option instead.")
  (option m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionNotification>) ostream)
  "Serializes a message object of type '<VisionNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionNotification>) istream)
  "Deserializes a message object of type '<VisionNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionNotification>)))
  "Returns string type for a message object of type '<VisionNotification>"
  "kortex_driver/VisionNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionNotification)))
  "Returns string type for a message object of type 'VisionNotification"
  "kortex_driver/VisionNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionNotification>)))
  "Returns md5sum for a message object of type '<VisionNotification>"
  "1894e81fd5e3bdb5e082271dc8a09dcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionNotification)))
  "Returns md5sum for a message object of type 'VisionNotification"
  "1894e81fd5e3bdb5e082271dc8a09dcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionNotification>)))
  "Returns full string definition for message of type '<VisionNotification>"
  (cl:format cl:nil "~%uint32 event~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionNotification)))
  "Returns full string definition for message of type 'VisionNotification"
  (cl:format cl:nil "~%uint32 event~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionNotification>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionNotification
    (cl:cons ':event (event msg))
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':option (option msg))
))
