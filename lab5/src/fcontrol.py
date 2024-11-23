""" ========== UniBo: AI and Robotics 2024 ==========

A class to build fuzzy controllers.  A fuzzy controller is defined by a set of fuzzy rules
together with the corresponding fuzzy predicates and fuzzy linguistic variables.

A fuzzy rule has the form
  { Name : (Antecedent, Linguistic variable, Linguistic value) }
where:
- Antecedent is an expression in fuzzy propositional logic, buid from a set
  of fuzzy predicates (see below) and the fuzzy connectives AND, OR, NOT.
  NOT is a prefix unary operator; AND and NOT are infix binary operators
- Linguistic variable is the name of a linguistic variable (see below)
- Linguistic value is a linguistic value for the linguistic variable

Example of a fuzzy rule:
  { 'ToLeft'  : ("TargetLeft AND NOT(TargetHere)", 'Turn', 'Left') }

A fuzzy predicate has the form
  { Name : (Membership function, Input variable) }
where:
- Membership function is a function returning a value in [0,1]
- Input variable is the argument to the membership function

Example of a fuzzy predicate:
  { 'TargetLeft'  : (ramp_down(-30.0, -5.0), 'phi') }
where ramp_down is a predefined membership function.

A linguistic variable (a.k.a. "fuzzy action") has the form
  { Name : (Linguistic values, Target variable)}
where:
- Linguistic values is a dictionary of {label : value}
- Target variable is the control variable to which the values refer to

Example of a linguistic variable:
  { 'Turn' : ({'Left':-30, 'MLeft':-10, 'None':0, 'MRight':10, 'Right':30}, 'Vrot') }

(c) 2024 Alessandro Saffiotti
"""
from collections import deque
from math import sin, cos, radians


class FController:
    """
    This class implements a generic fuzzy rule-based controller
    It provides the basic mechanisms for the "FBehavior" class
    It uses the "FEval" class to evaluate the truth value of the rule antecedents
    """
    state  = {}     # input state, used to compute all the truth values
    output = {}     # output variables, computed by the fuzzy controller
    frules = {}     # fuzzy rules
    fpreds = {}     # fuzzy predicates, used in the LHS of the rules
    flvars = {}     # fuzzy linguistic variables, used in the RHS of the rules
    flsets = {}     # fuzzy sets, hold the values of the linguistic variables
    fpvals = {}     # truth values of the fuzzy pedicates
    fgoal  = ""     # goal condition, as a statement in fuzzy logic

    def __init__(self):
        self.fevaluator = FEval()

    def init_flsets(self):
        for name in self.flvars:
            flvar = self.flvars[name]
            cvar  = flvar[1]
            self.flsets[cvar] = ({}, name)
            self.output[cvar] = 0.0
            for label in flvar[0]:
                self.flsets[cvar][0][label] = 0.0

    def init_fpreds(self):
        for pred in self.fpreds:
            self.fpvals[pred] = 0.0

    def eval_fpred(self, name):
        fpred = self.fpreds[name]
        return(fpred[0](self.state[fpred[1]]))
    
    def eval_fpreds(self):
        for pred in self.fpreds:
            self.fpvals[pred] = self.eval_fpred(pred)
        return self.fpvals

    def eval_frule(self, name, debug = 0):
        frule = self.frules[name]
        ante  = frule[0]
        flvar = self.flvars[frule[1]]
        label = frule[2]
        cvar  = flvar[1]
        flset = self.flsets[cvar][0]
        level = self.fevaluator.eval(ante)
        flset[label] = max(flset[label], level)
        if debug > 0:
            print('  {} [{:.2f}] -> {}'.format(name, level, frule))
            # print("  {} -> {}".format(cvar, flset))

    def eval_frules(self, debug = 0):
        if debug > 0:
            self.print_fpreds()
        self.init_flsets()
        if debug > 0:
            print('Rules:')
        for frule in self.frules:
            self.eval_frule(frule, debug)

    def eval_goal(self, debug = 0):
        return self.fevaluator.eval(self.fgoal)

    def defuzzify_var(self, cvar, debug = 0):
        lvar = self.flsets[cvar][1]
        vals = self.flvars[lvar][0]
        mus  = self.flsets[cvar][0]
        sumv = 0.0
        sumu = 0.0
        if debug > 2:
            print("defuzzify:", cvar)
            print("  vals:", vals)
            print("  mu's:", mus)
        for label in vals:
            sumv += mus[label] * vals[label]
            sumu += mus[label]
        if sumu == 0.0:
            return None
        return sumv / sumu
    
    def defuzzify(self, debug = 0):
        for cvar in self.output:
            val = self.defuzzify_var(cvar, debug)
            if val is not None:
                self.output[cvar] = val

    def run(self, state, debug = 0):
        self.update_state(state)
        if debug > 0:
            self.print_state()
        self.eval_fpreds()
        self.fevaluator.set_interpretation(self.fpvals)
        self.eval_frules(debug)
        self.defuzzify(debug)
        return True, self.eval_goal()
    
    def get_output(self):
        return self.output

    def print_state(self):
        print("Input state:")
        for var in self.state:
            print('  {}: {:.3f}'.format(var, self.state[var]))
    
    def print_output(self):
        print("Output variables:")
        for var in self.output:
            print('  {}: {:.3f}'.format(var, self.output[var]))
    
    def print_fpreds(self):
        print("Fuzzy predicates:")
        for pred in self.fpreds:
            print('  {}: {:.3f}'.format(pred, self.eval_fpred(pred)))

    def print_flvars(self):
        print("Fuzzy linguistic variables:")
        for name in self.flvars:
            flvar = self.flvars[name]
            print('', name + ':', flvar[0], '(' + flvar[1] + ')')

    def print_flsets(self):
        print("Fuzzy linguistic sets:")
        for name in self.flsets:
            flset = self.flsets[name]
            print('', name + ':', flset[0], '(' + flset[1] + ')')

    def print_frules(self):
        print("Fuzzy Rules:")
        for name in self.frules:
            frule = self.frules[name]
            print("{: <12}".format(name + ": "), end = "")
            print("IF " + "{: <50}".format(frule[0]) + " THEN " + frule[1] + "(" + frule[2] + ")")
    
    def print_all(self):
        self.print_fpreds()
        self.print_flvars()
        self.print_flsets()
        self.print_frules()
        self.print_state()
        self.print_output()


class Behavior (FController):
    """
    Top level class for defining a fuzzy "behavior" using a fuzzy rule-based controller
    Behaviors are specialized fuzzy controllers that realize one specific task.
    Each is created by defining its specific rules, fuzzy predicates and fuzzy
    linguistic variables (actions), in the function "setup"; as well as the specific
    way to compute the relevant state variables, in the function "update_state".
    Each specific behavior, like "GoToTarget", must be a sub-classe of this one,
    and it must specialize these two functions:
    - setup (where all fuzzy predicates, actions and rules are defined)
    - update_state (how to compute the relevant state variables)
    All behaviors have two control variables: 'Vlin' and 'Vrot'
    """

    def __init__(self):
        super().__init__()
        self.setup()

    def setup(self):
        """
        Define the set of fuzzy rules, fuzzy predicates and fuzzy linguistic variables
        Must be instantiated for any specific subclass of FBehavior
        """
        pass

    def update_state(self, state):
        """
        Set the local variables depending on the current robot's state
        Must be instantiated for any specific subclass of FBehavior
        """
        pass

    def get_vlin(self):
        return self.output['Vlin']

    def get_vrot(self):
        return radians(self.output['Vrot'])


class FEval:
    """
    A class to evaluate the truth value of statement in propositional fuzzy logic,
    using a simple LR(1) parser based on the following BNF:
      <stmt> ::= <term> | <term> "OR" <term> | <term> "AND" <term>
      <term> ::= <atom> | <encl> | "NOT" <encl>
      <encl> ::= "(" <stmt> ")"
    Truth values are computed with respect to a given interpretation, that is, an
    assignment of a truth value to each fuzzy predicate, given as a dict 'fpreds'.
    """
    def __init__(self, debug=0):
        self.debug  = debug
        self.stmt   = ""
        self.input  = None
        self.fpreds = {}
        self.nest   = 1

    def set_interpretation(self, dict):
        self.fpreds = dict

    def this(self):
        if len(self.input) < 1:
            return None
        return self.input[0]

    def next(self):
        if len(self.input) < 2:
            return None
        return self.input[1]

    def eval(self, stmt, debug = 0):
        """
        Compute the truth value of a fuzzy statement 'stmt', given as a string
        Assume that a fuzzy interpretation has been set through set_interpretation
        """
        self.expr  = stmt
        self.input = deque(stmt.replace('(',' ( ').replace(')',' ) ').split())
        self.value = 0.0
        result = self.parse_stmt()
        if debug > 3:
            print("> FEval interp:", self.fpreds)
            print("> FEval input:", self.input)
            print("> FEval value:", result)
        assert result != None, "Invalid syntax in fuzzy expression: " + self.expr
        return result
    
    def parse_stmt(self):
        self.debug_enter("stmt")
        lhs = self.parse_term()
        if lhs == None:
            self.debug_exit("stmt", None)
            return None
        op = self.this()
        if op == 'AND':
            self.input.popleft()
            rhs = self.parse_term()
            if rhs == None:
                self.debug_exit("stmt", None)
                return None
            result = self.eval_and(lhs, rhs)
        elif op == 'OR':
            self.input.popleft()
            rhs = self.parse_term()
            if rhs == None:
                self.debug_exit("stmt", None)
                return None
            result = self.eval_or(lhs, rhs)
            self.debug_exit("stmt", result)
            return result
        else:
            result = lhs
        self.debug_exit("stmt", result)
        return result
    
    def parse_term(self):
        self.debug_enter("term")
        if self.this() == 'NOT':
            self.input.popleft()
            term = self.parse_encl()
            if term == None:
                self.debug_exit("term", None)
                return None
            result = self.eval_not(term)
        elif self.is_atom(self.this()):
            result = self.parse_atom()
        else:
            result = self.parse_encl()
        self.debug_exit("term", result)
        return result
  
    def parse_encl(self):
        self.debug_enter("encl")
        if self.this() == '(':
            self.input.popleft()
            expr = self.parse_stmt()
            if expr == None:
                self.debug_exit("encl", None)
                return None
            if self.this() == ')':
                self.input.popleft()
                result = self.eval_paren(expr)
                self.debug_exit("encl", result)
                return result
        self.debug_exit("encl", None)
        return None
  
    def parse_atom(self):
        self.debug_enter("atom")
        token = self.input.popleft()
        if self.is_atom(token):
            result = self.eval_pred(token)
            self.debug_exit("atom", result)
            return result
        self.debug_exit("atom", None)
        return None
    
    def is_atom(self, token):
        if token in ['NOT', 'AND', 'OR', '(', ')']:
            return False
        return True
    
    def debug_enter(self, category):
        self.nest = self.nest + 1
        if self.debug > 1:
            print("".ljust(self.nest), category, ">", list(self.input))

    def debug_exit(self, category, val):
        self.nest = self.nest - 1
        if self.debug > 1:
            print("".ljust(self.nest), category, "<", val)

    def eval_pred(self, pred):
        if pred not in self.fpreds:
            raise LookupError("No truth value provided for fuzzy predicate: " + pred)
        if self.debug == 0:
            return self.fpreds[pred]
        else:
            return '[' + str(self.fpreds[pred]) + ']'

    def eval_and(self, lhs, rhs):
        if self.debug == 0:
            return min(lhs, rhs)
        else:
            return 'min(' + lhs + ', ' + rhs
    
    def eval_or(self, lhs, rhs):
        if self.debug == 0:
            return max(lhs, rhs)
        else:
            return 'max(' + lhs + ', ' + rhs
    
    def eval_not(self, term):
        if self.debug == 0:
            return 1.0 - term
        else:
            return '1 - ' + term

    def eval_paren(self, stmt):
        if self.debug == 0:
            return stmt
        else:
            return '(' + stmt + ')'


"""
Some common builders for membership functions
"""

def ramp_up(a, b):
    """
    zero until a, then raise linearly until b, then one
    """
    assert a <= b, f"ramp_up called with a > b ({a}, {b})"
    def mu(x):
        if x <= a:
            return 0.0
        if x > b:
            return 1.0
        return (x-a)/(b-a)
    return mu


def ramp_down(a, b):
    """
    one until a, then decrease linearly until b, then zero
    """
    assert a <= b, f"ramp_down called with a > b ({a}, {b})"
    def mu(x):
        if x <= a:
            return 1.0
        if x > b:
            return 0.0
        return (b-x)/(b-a)
    return mu


def triangle(a, b, c):
    """
    zero until a, then raise linearly until b, then decrease linearly until c, then zero
    """
    assert a <= b, f"triangle called with a > b ({a}, {b})"
    assert b <= c, f"triangle called with b > c ({b}, {c})"
    def mu(x):
        if x <= a:
            return 0.0
        if x > c:
            return 0.0
        if x <= b:
            return (x-a)/(b-a)
        return (c-x)/(c-b)
    return mu


def trapezoid(a, b, c, d):
    """
    zero until a, raise linearly until b, one unti c, decrease linearly until d, then zero
    """
    assert a <= b, f"trapezoid called with a > b ({a}, {b})"
    assert b <= c, f"trapezoid called with b > c ({b}, {c})"
    assert c <= d, f"trapezoid called with c > d ({c}, {d})"
    def mu(x):
        if x <= a:
            return 0.0
        if x > d:
            return 0.0
        if x <= b:
            return (x-a)/(b-a)
        if x <= c:
            return 1.0
        return (d-x)/(d-c)
    return mu

"""
Some basic geometry
"""

def global_to_local(pose, frame, newpose):
    """
    Transform pose from global to local frame, store it in newpose
    Frame are coordinates of local frame in global one
    Poses are triples [x, y, th], in mt and rad
    """
    x  = pose[0]
    y  = pose[1]
    th = pose[2]
    x0 = frame[0]
    y0 = frame[1]
    th0 = frame[2]
    newpose[0] = x * cos(th0) + y * sin(th0) - (x0 * cos(th0) + y0 * sin(th0))
    newpose[1] = y * cos(th0) - x * sin(th0) + (x0 * sin(th0) - y0 * cos(th0))
    newpose[2] = th - th0
        
def local_to_global(pose, frame, newpose):
    """
    Transform pose from local to global frame, store it in newpose
    Frame are coordinates of local frame in global one
    Poses are triples [x, y, th], in mt and rad
    """
    x  = pose[0]
    y  = pose[1]
    th = pose[2]
    x0 = frame[0]
    y0 = frame[1]
    th0 = frame[2]
    newpose[0] = x * cos(th0) - y * sin(th0) + x0
    newpose[1] = x * sin(th0) + y * cos(th0) + y0
    newpose[2] = th + th0

