; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorCyclic_CustomData.msg.html

(cl:defclass <ActuatorCyclic_CustomData> (roslisp-msg-protocol:ros-message)
  ((custom_data_id
    :reader custom_data_id
    :initarg :custom_data_id
    :type kortex_driver-msg:ActuatorCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:ActuatorCyclic_MessageId))
   (custom_data_0
    :reader custom_data_0
    :initarg :custom_data_0
    :type cl:integer
    :initform 0)
   (custom_data_1
    :reader custom_data_1
    :initarg :custom_data_1
    :type cl:integer
    :initform 0)
   (custom_data_2
    :reader custom_data_2
    :initarg :custom_data_2
    :type cl:integer
    :initform 0)
   (custom_data_3
    :reader custom_data_3
    :initarg :custom_data_3
    :type cl:integer
    :initform 0)
   (custom_data_4
    :reader custom_data_4
    :initarg :custom_data_4
    :type cl:integer
    :initform 0)
   (custom_data_5
    :reader custom_data_5
    :initarg :custom_data_5
    :type cl:integer
    :initform 0)
   (custom_data_6
    :reader custom_data_6
    :initarg :custom_data_6
    :type cl:integer
    :initform 0)
   (custom_data_7
    :reader custom_data_7
    :initarg :custom_data_7
    :type cl:integer
    :initform 0)
   (custom_data_8
    :reader custom_data_8
    :initarg :custom_data_8
    :type cl:integer
    :initform 0)
   (custom_data_9
    :reader custom_data_9
    :initarg :custom_data_9
    :type cl:integer
    :initform 0)
   (custom_data_10
    :reader custom_data_10
    :initarg :custom_data_10
    :type cl:integer
    :initform 0)
   (custom_data_11
    :reader custom_data_11
    :initarg :custom_data_11
    :type cl:integer
    :initform 0)
   (custom_data_12
    :reader custom_data_12
    :initarg :custom_data_12
    :type cl:integer
    :initform 0)
   (custom_data_13
    :reader custom_data_13
    :initarg :custom_data_13
    :type cl:integer
    :initform 0)
   (custom_data_14
    :reader custom_data_14
    :initarg :custom_data_14
    :type cl:integer
    :initform 0)
   (custom_data_15
    :reader custom_data_15
    :initarg :custom_data_15
    :type cl:integer
    :initform 0))
)

(cl:defclass ActuatorCyclic_CustomData (<ActuatorCyclic_CustomData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorCyclic_CustomData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorCyclic_CustomData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorCyclic_CustomData> is deprecated: use kortex_driver-msg:ActuatorCyclic_CustomData instead.")))

(cl:ensure-generic-function 'custom_data_id-val :lambda-list '(m))
(cl:defmethod custom_data_id-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_id-val is deprecated.  Use kortex_driver-msg:custom_data_id instead.")
  (custom_data_id m))

(cl:ensure-generic-function 'custom_data_0-val :lambda-list '(m))
(cl:defmethod custom_data_0-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_0-val is deprecated.  Use kortex_driver-msg:custom_data_0 instead.")
  (custom_data_0 m))

(cl:ensure-generic-function 'custom_data_1-val :lambda-list '(m))
(cl:defmethod custom_data_1-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_1-val is deprecated.  Use kortex_driver-msg:custom_data_1 instead.")
  (custom_data_1 m))

(cl:ensure-generic-function 'custom_data_2-val :lambda-list '(m))
(cl:defmethod custom_data_2-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_2-val is deprecated.  Use kortex_driver-msg:custom_data_2 instead.")
  (custom_data_2 m))

(cl:ensure-generic-function 'custom_data_3-val :lambda-list '(m))
(cl:defmethod custom_data_3-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_3-val is deprecated.  Use kortex_driver-msg:custom_data_3 instead.")
  (custom_data_3 m))

(cl:ensure-generic-function 'custom_data_4-val :lambda-list '(m))
(cl:defmethod custom_data_4-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_4-val is deprecated.  Use kortex_driver-msg:custom_data_4 instead.")
  (custom_data_4 m))

(cl:ensure-generic-function 'custom_data_5-val :lambda-list '(m))
(cl:defmethod custom_data_5-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_5-val is deprecated.  Use kortex_driver-msg:custom_data_5 instead.")
  (custom_data_5 m))

(cl:ensure-generic-function 'custom_data_6-val :lambda-list '(m))
(cl:defmethod custom_data_6-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_6-val is deprecated.  Use kortex_driver-msg:custom_data_6 instead.")
  (custom_data_6 m))

(cl:ensure-generic-function 'custom_data_7-val :lambda-list '(m))
(cl:defmethod custom_data_7-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_7-val is deprecated.  Use kortex_driver-msg:custom_data_7 instead.")
  (custom_data_7 m))

(cl:ensure-generic-function 'custom_data_8-val :lambda-list '(m))
(cl:defmethod custom_data_8-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_8-val is deprecated.  Use kortex_driver-msg:custom_data_8 instead.")
  (custom_data_8 m))

(cl:ensure-generic-function 'custom_data_9-val :lambda-list '(m))
(cl:defmethod custom_data_9-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_9-val is deprecated.  Use kortex_driver-msg:custom_data_9 instead.")
  (custom_data_9 m))

(cl:ensure-generic-function 'custom_data_10-val :lambda-list '(m))
(cl:defmethod custom_data_10-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_10-val is deprecated.  Use kortex_driver-msg:custom_data_10 instead.")
  (custom_data_10 m))

(cl:ensure-generic-function 'custom_data_11-val :lambda-list '(m))
(cl:defmethod custom_data_11-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_11-val is deprecated.  Use kortex_driver-msg:custom_data_11 instead.")
  (custom_data_11 m))

(cl:ensure-generic-function 'custom_data_12-val :lambda-list '(m))
(cl:defmethod custom_data_12-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_12-val is deprecated.  Use kortex_driver-msg:custom_data_12 instead.")
  (custom_data_12 m))

(cl:ensure-generic-function 'custom_data_13-val :lambda-list '(m))
(cl:defmethod custom_data_13-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_13-val is deprecated.  Use kortex_driver-msg:custom_data_13 instead.")
  (custom_data_13 m))

(cl:ensure-generic-function 'custom_data_14-val :lambda-list '(m))
(cl:defmethod custom_data_14-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_14-val is deprecated.  Use kortex_driver-msg:custom_data_14 instead.")
  (custom_data_14 m))

(cl:ensure-generic-function 'custom_data_15-val :lambda-list '(m))
(cl:defmethod custom_data_15-val ((m <ActuatorCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_15-val is deprecated.  Use kortex_driver-msg:custom_data_15 instead.")
  (custom_data_15 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorCyclic_CustomData>) ostream)
  "Serializes a message object of type '<ActuatorCyclic_CustomData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'custom_data_id) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_9)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_9)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_9)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_9)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_10)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_10)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_10)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_10)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_11)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_11)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_11)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_11)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_12)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_12)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_12)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_12)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_13)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_13)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_13)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_13)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_14)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_14)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_14)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_14)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_15)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_15)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_15)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_15)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorCyclic_CustomData>) istream)
  "Deserializes a message object of type '<ActuatorCyclic_CustomData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'custom_data_id) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_9)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_9)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_9)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_9)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_10)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_10)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_10)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_10)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_11)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_11)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_11)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_11)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_12)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_12)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_12)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_12)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_13)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_13)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_13)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_13)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_14)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_14)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_14)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_14)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_data_15)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_data_15)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'custom_data_15)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'custom_data_15)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorCyclic_CustomData>)))
  "Returns string type for a message object of type '<ActuatorCyclic_CustomData>"
  "kortex_driver/ActuatorCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorCyclic_CustomData)))
  "Returns string type for a message object of type 'ActuatorCyclic_CustomData"
  "kortex_driver/ActuatorCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorCyclic_CustomData>)))
  "Returns md5sum for a message object of type '<ActuatorCyclic_CustomData>"
  "05eca88f613f992646a3cd017b934eba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorCyclic_CustomData)))
  "Returns md5sum for a message object of type 'ActuatorCyclic_CustomData"
  "05eca88f613f992646a3cd017b934eba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorCyclic_CustomData>)))
  "Returns full string definition for message of type '<ActuatorCyclic_CustomData>"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId custom_data_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorCyclic_CustomData)))
  "Returns full string definition for message of type 'ActuatorCyclic_CustomData"
  (cl:format cl:nil "~%ActuatorCyclic_MessageId custom_data_id~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%================================================================================~%MSG: kortex_driver/ActuatorCyclic_MessageId~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorCyclic_CustomData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'custom_data_id))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorCyclic_CustomData>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorCyclic_CustomData
    (cl:cons ':custom_data_id (custom_data_id msg))
    (cl:cons ':custom_data_0 (custom_data_0 msg))
    (cl:cons ':custom_data_1 (custom_data_1 msg))
    (cl:cons ':custom_data_2 (custom_data_2 msg))
    (cl:cons ':custom_data_3 (custom_data_3 msg))
    (cl:cons ':custom_data_4 (custom_data_4 msg))
    (cl:cons ':custom_data_5 (custom_data_5 msg))
    (cl:cons ':custom_data_6 (custom_data_6 msg))
    (cl:cons ':custom_data_7 (custom_data_7 msg))
    (cl:cons ':custom_data_8 (custom_data_8 msg))
    (cl:cons ':custom_data_9 (custom_data_9 msg))
    (cl:cons ':custom_data_10 (custom_data_10 msg))
    (cl:cons ':custom_data_11 (custom_data_11 msg))
    (cl:cons ':custom_data_12 (custom_data_12 msg))
    (cl:cons ':custom_data_13 (custom_data_13 msg))
    (cl:cons ':custom_data_14 (custom_data_14 msg))
    (cl:cons ':custom_data_15 (custom_data_15 msg))
))
