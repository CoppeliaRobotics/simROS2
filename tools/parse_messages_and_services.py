from sys import argv, exit, stderr
import os
import re
import subprocess

class TypeSpec:
    '''
    parse a type specification, such as Header, geometry_msgs/Point, or string[12]
    '''

    fast_write_types = {'int32': 'Int32', 'float32': 'Float', 'float64': 'Double'}

    ctype_builtin = {
            'bool':         'uint8_t',
            'int8':         'int8_t',
            'uint8':        'uint8_t',
            'int16':        'int16_t',
            'uint16':       'uint16_t',
            'int32':        'int32_t',
            'uint32':       'uint32_t',
            'int64':        'int64_t',
            'uint64':       'uint64_t',
            'float32':      'float',
            'float64':      'double',
            'string':       'std::string',
            'time':         'ros::Time',
            'duration':     'ros::Duration',
            'byte':         'uint8_t',
            'char':         'int8_t'
    }

    deprecated_builtins = {
    }

    def __init__(self, s, tag, parent=None):
        if tag not in ('msg', 'srv'):
            raise ValueError('tag must be "msg" or "srv"')
        self.tag = tag
        def is_identifier(s):
            return re.match('^[a-zA-Z_][a-zA-Z0-9_]*$', s)
        self.array = False
        self.array_size = None
        m = re.match(r'^(.*)\[(\d*)\]$', s)
        if m:
            self.array = True
            s = m.group(1)
            if len(m.group(2)) > 0:
                self.array_size = int(m.group(2))
        # perform substitutions:
        if s in self.deprecated_builtins: s = self.deprecated_builtins[s]
        # check builtins:
        self.builtin = s in self.ctype_builtin
        self.fullname = s
        tok = s.split('/')
        if self.builtin:
            self.package = 'builtin'
            self.mtype = s
        elif len(tok) == 2:
            self.package = tok[0]
            self.mtype = tok[1]
        elif len(tok) == 1 and parent is not None:
            self.package = parent.package
            self.mtype = tok[0]
        else:
            raise ValueError('bad type: %s' % s)
        if not is_identifier(self.package) or not is_identifier(self.mtype):
            raise ValueError('bad type: %s (%s/%s)' % (s, self.package, self.mtype))

    # normalize fullname to C identifier (replace / with __)
    def normalized(self):
        return ('{}__'.format(self.package) if not self.builtin else '') + self.mtype

    # get C++ type declaration
    def cpp_type(self):
        if self.builtin: return self.ctype_builtin[self.mtype]
        r = '::'.join([self.package] + [self.tag][int(self.tag is None):] + [self.mtype])
        r = re.sub(r'^([\w:]+)(Request|Response)$', r'\1::\2', r)
        return r

    # get C++ include header name
    def cpp_include(self):
        return '{}/{}/{}.hpp'.format(*map(camel_case_to_snake_case, (self.package, self.tag, self.mtype)))

    def __str__(self):
        t = self.mtype
        if not self.builtin:
            t = self.package + '/' + t
        if self.array:
            t += '[]'
        return t

class Fields:
    def __init__(self, ts, lines, subtype=''):
        # parse msg / srv definition
        self.fields = {}

        for ln in lines:
            if ln.startswith('  '):
                # ignore expansions of nested types
                continue

            ln_orig1 = ln

            # strip comments
            ln = re.sub('#.*$', '', ln).strip()

            # strip bounded arrays
            ln = re.sub(r'\[<=\d+\]', '[]', ln)

            # strip bounded strings
            ln = re.sub(r'string<=\d+', 'string', ln)

            if not ln:
                # ignore empty lines
                continue

            ln = ln.replace('=',' = ')

            tokens = ln.split()

            if len(tokens) == 4 and tokens[2] == '=':
                # it's a constant definition: ignore
                continue

            if len(tokens) in (2, 3):
                # 3rd token is the default value, unused for now
                t = TypeSpec(tokens[0], 'msg', parent=ts)
                n = tokens[1]
                self.fields[n] = t
                continue

            raise ValueError('unrecognized line: "%s"' % ln_orig1)

def camel_case_to_snake_case(x):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', x)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

class MsgInfo:
    def __init__(self, msg_name, typespec=None, fields=None):
        if typespec is None:
            self.typespec = TypeSpec(msg_name, 'msg')
        else:
            self.typespec = typespec
        if fields is None:
            print('Reading definition of msg {}...'.format(msg_name))
            # parse msg definition
            cmd = ['ros2', 'msg', 'show', msg_name]
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            if p.wait() != 0:
                raise Exception('execution of %s failed' % cmd)
            out, err = p.communicate()
            lines = out.splitlines()
            msg = Fields(self.typespec, lines)
            self.fields = msg.fields
        else:
            self.fields = fields

class SrvInfo:
    def __init__(self, srv_name):
        print('Reading definition of srv {}...'.format(srv_name))
        # parse srv definition
        cmd = ['ros2', 'srv', 'show', srv_name]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        if p.wait() != 0:
            raise Exception('execution of %s failed' % cmd)
        out, err = p.communicate()
        lines = out.splitlines()
        sep = '---'
        if sep not in lines:
            raise ValueError('bad srv definition')
        i = lines.index(sep)
        self.typespec = TypeSpec(srv_name, 'srv')
        self.req = Fields(self.typespec, lines[:i], 'Request')
        self.resp = Fields(self.typespec, lines[i+1:], 'Response')

def get_msgs_info(messages_file):
    # populate msg list
    msg_list = set()
    with open(messages_file) as f:
        for l in f.readlines():
            l = re.sub('#.*$', '', l).strip()
            if not l: continue
            msg_list.add(l)

    # get msg definitions
    msg_fields = {}
    for msg in sorted(msg_list):
        try:
            msg_fields[msg] = MsgInfo(msg)
        except Exception as e:
            print('WARNING: bad msg: %s (%s)' % (msg, e.args[0]))
            continue

    return msg_fields

def get_srvs_info(services_file):
    # populate srv list
    srv_list = set()
    with open(services_file) as f:
        for l in f.readlines():
            l = re.sub('#.*$', '', l).strip()
            if not l: continue
            srv_list.add(l)

    # get srv definitions
    srv_fields = {}
    for srv in sorted(srv_list):
        try:
            srv_fields[srv] = SrvInfo(srv)
        except Exception as e:
            print('WARNING: bad srv: %s (%s)' % (srv, e.args[0]))
            continue

    return srv_fields

def get_msgs_srvs_info(messages_file, services_file):
    msg_fields = get_msgs_info(messages_file)
    srv_fields = get_srvs_info(services_file)
    get_msgs_srvs_info_(msg_fields, srv_fields)

def get_msgs_srvs_info_(msg_fields, srv_fields):
    ret = msg_fields.copy()
    for srv, info in srv_fields.items():
        for k, v in {'Request': info.req.fields, 'Response': info.resp.fields}.items():
            msg = srv + k
            try:
                ret[msg] = MsgInfo(msg, typespec=TypeSpec(msg, 'srv'), fields=v)
            except Exception as e:
                print('WARNING: bad msg: %s (%s)' % (srv, e.args[0]))
                continue
    return ret

def load_cache(pickle_file):
    import pickle
    with open(pickle_file, 'rb') as f:
        return pickle.load(f)

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 4:
        print('usage: {} <messages_file> <services_file> <pickle_output_file>'.format(sys.argv[0]))
        sys.exit(1)
    for c in (MsgInfo, SrvInfo, TypeSpec, Fields):
        c.__module__ = 'parse_messages_and_services'
    messages_file, services_file, output_file = sys.argv[1:]
    msgs = get_msgs_info(messages_file)
    srvs = get_srvs_info(services_file)
    msgssrvs = get_msgs_srvs_info_(msgs, srvs)
    data = (msgs, srvs, msgssrvs)
    import pickle
    with open(output_file, 'wb') as f:
        pickle.dump(data, f)


