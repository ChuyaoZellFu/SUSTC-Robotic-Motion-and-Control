; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Connection.msg.html

(cl:defclass <Connection> (roslisp-msg-protocol:ros-message)
  ((user_handle
    :reader user_handle
    :initarg :user_handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (connection_information
    :reader connection_information
    :initarg :connection_information
    :type cl:string
    :initform "")
   (connection_identifier
    :reader connection_identifier
    :initarg :connection_identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass Connection (<Connection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Connection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Connection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Connection> is deprecated: use kortex_driver-msg:Connection instead.")))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <Connection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection_information-val :lambda-list '(m))
(cl:defmethod connection_information-val ((m <Connection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection_information-val is deprecated.  Use kortex_driver-msg:connection_information instead.")
  (connection_information m))

(cl:ensure-generic-function 'connection_identifier-val :lambda-list '(m))
(cl:defmethod connection_identifier-val ((m <Connection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection_identifier-val is deprecated.  Use kortex_driver-msg:connection_identifier instead.")
  (connection_identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Connection>) ostream)
  "Serializes a message object of type '<Connection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'connection_information))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'connection_information))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'connection_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'connection_identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Connection>) istream)
  "Deserializes a message object of type '<Connection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'connection_information) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'connection_information) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'connection_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'connection_identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Connection>)))
  "Returns string type for a message object of type '<Connection>"
  "kortex_driver/Connection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Connection)))
  "Returns string type for a message object of type 'Connection"
  "kortex_driver/Connection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Connection>)))
  "Returns md5sum for a message object of type '<Connection>"
  "149818e986cd66f3e32843abf7b3bfe5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Connection)))
  "Returns md5sum for a message object of type 'Connection"
  "149818e986cd66f3e32843abf7b3bfe5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Connection>)))
  "Returns full string definition for message of type '<Connection>"
  (cl:format cl:nil "~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Connection)))
  "Returns full string definition for message of type 'Connection"
  (cl:format cl:nil "~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Connection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     4 (cl:length (cl:slot-value msg 'connection_information))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Connection>))
  "Converts a ROS message object to a list"
  (cl:list 'Connection
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection_information (connection_information msg))
    (cl:cons ':connection_identifier (connection_identifier msg))
))
