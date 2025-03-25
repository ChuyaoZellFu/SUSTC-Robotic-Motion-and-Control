; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Query.msg.html

(cl:defclass <Query> (roslisp-msg-protocol:ros-message)
  ((start_timestamp
    :reader start_timestamp
    :initarg :start_timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (end_timestamp
    :reader end_timestamp
    :initarg :end_timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (username
    :reader username
    :initarg :username
    :type cl:string
    :initform ""))
)

(cl:defclass Query (<Query>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Query>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Query)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Query> is deprecated: use kortex_driver-msg:Query instead.")))

(cl:ensure-generic-function 'start_timestamp-val :lambda-list '(m))
(cl:defmethod start_timestamp-val ((m <Query>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:start_timestamp-val is deprecated.  Use kortex_driver-msg:start_timestamp instead.")
  (start_timestamp m))

(cl:ensure-generic-function 'end_timestamp-val :lambda-list '(m))
(cl:defmethod end_timestamp-val ((m <Query>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:end_timestamp-val is deprecated.  Use kortex_driver-msg:end_timestamp instead.")
  (end_timestamp m))

(cl:ensure-generic-function 'username-val :lambda-list '(m))
(cl:defmethod username-val ((m <Query>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:username-val is deprecated.  Use kortex_driver-msg:username instead.")
  (username m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Query>) ostream)
  "Serializes a message object of type '<Query>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_timestamp) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'username))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'username))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Query>) istream)
  "Deserializes a message object of type '<Query>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_timestamp) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'username) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'username) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Query>)))
  "Returns string type for a message object of type '<Query>"
  "kortex_driver/Query")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Query)))
  "Returns string type for a message object of type 'Query"
  "kortex_driver/Query")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Query>)))
  "Returns md5sum for a message object of type '<Query>"
  "13ee5e330f26cd5fd95364d08fb2a910")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Query)))
  "Returns md5sum for a message object of type 'Query"
  "13ee5e330f26cd5fd95364d08fb2a910")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Query>)))
  "Returns full string definition for message of type '<Query>"
  (cl:format cl:nil "~%Timestamp start_timestamp~%Timestamp end_timestamp~%string username~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Query)))
  "Returns full string definition for message of type 'Query"
  (cl:format cl:nil "~%Timestamp start_timestamp~%Timestamp end_timestamp~%string username~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Query>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_timestamp))
     4 (cl:length (cl:slot-value msg 'username))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Query>))
  "Converts a ROS message object to a list"
  (cl:list 'Query
    (cl:cons ':start_timestamp (start_timestamp msg))
    (cl:cons ':end_timestamp (end_timestamp msg))
    (cl:cons ':username (username msg))
))
