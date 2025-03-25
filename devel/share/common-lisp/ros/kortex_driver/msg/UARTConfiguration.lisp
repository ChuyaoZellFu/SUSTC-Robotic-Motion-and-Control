; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTConfiguration.msg.html

(cl:defclass <UARTConfiguration> (roslisp-msg-protocol:ros-message)
  ((port_id
    :reader port_id
    :initarg :port_id
    :type cl:integer
    :initform 0)
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (word_length
    :reader word_length
    :initarg :word_length
    :type cl:integer
    :initform 0)
   (stop_bits
    :reader stop_bits
    :initarg :stop_bits
    :type cl:integer
    :initform 0)
   (parity
    :reader parity
    :initarg :parity
    :type cl:integer
    :initform 0))
)

(cl:defclass UARTConfiguration (<UARTConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTConfiguration> is deprecated: use kortex_driver-msg:UARTConfiguration instead.")))

(cl:ensure-generic-function 'port_id-val :lambda-list '(m))
(cl:defmethod port_id-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_id-val is deprecated.  Use kortex_driver-msg:port_id instead.")
  (port_id m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:speed-val is deprecated.  Use kortex_driver-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'word_length-val :lambda-list '(m))
(cl:defmethod word_length-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:word_length-val is deprecated.  Use kortex_driver-msg:word_length instead.")
  (word_length m))

(cl:ensure-generic-function 'stop_bits-val :lambda-list '(m))
(cl:defmethod stop_bits-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:stop_bits-val is deprecated.  Use kortex_driver-msg:stop_bits instead.")
  (stop_bits m))

(cl:ensure-generic-function 'parity-val :lambda-list '(m))
(cl:defmethod parity-val ((m <UARTConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:parity-val is deprecated.  Use kortex_driver-msg:parity instead.")
  (parity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTConfiguration>) ostream)
  "Serializes a message object of type '<UARTConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'word_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'word_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'word_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'word_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stop_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stop_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stop_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'parity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'parity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'parity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'parity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTConfiguration>) istream)
  "Deserializes a message object of type '<UARTConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'word_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'word_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'word_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'word_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stop_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stop_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stop_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'parity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'parity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'parity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'parity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTConfiguration>)))
  "Returns string type for a message object of type '<UARTConfiguration>"
  "kortex_driver/UARTConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTConfiguration)))
  "Returns string type for a message object of type 'UARTConfiguration"
  "kortex_driver/UARTConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTConfiguration>)))
  "Returns md5sum for a message object of type '<UARTConfiguration>"
  "da8e919001edf0734e1795af994fc4e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTConfiguration)))
  "Returns md5sum for a message object of type 'UARTConfiguration"
  "da8e919001edf0734e1795af994fc4e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTConfiguration>)))
  "Returns full string definition for message of type '<UARTConfiguration>"
  (cl:format cl:nil "~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTConfiguration)))
  "Returns full string definition for message of type 'UARTConfiguration"
  (cl:format cl:nil "~%uint32 port_id~%bool enabled~%uint32 speed~%uint32 word_length~%uint32 stop_bits~%uint32 parity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTConfiguration>))
  (cl:+ 0
     4
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTConfiguration
    (cl:cons ':port_id (port_id msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':word_length (word_length msg))
    (cl:cons ':stop_bits (stop_bits msg))
    (cl:cons ':parity (parity msg))
))
