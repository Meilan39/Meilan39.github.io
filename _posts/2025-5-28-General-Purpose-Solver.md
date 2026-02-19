---
title: "汎用数値最適化ソルバー"
date: 2025-02-19
categories: projects
mathjax: true
classes: wide
---

As technology continues to advance, things that humans could only dream of accomplishing a mere decade ago are now being achieved at breakneck speeds. However, being able to do 'anything' introduces new problems starting with sustainability. For example, the past decade has seen much push for greater efficiency in resource management, tackling environmental problems such as climate change. In an age where anything is possible, optimizing our approach to create a better future becomes a point of interest. 

Mathematical optimization is a sub-field of applied mathematics and computer science, with the goal of optimizing certain variables under a given constraint. More specifically, optimization often involves minimizing or maximizing a cost function, ensuring that any other constraints are being met simultaneously. It can be assumed that methods and technologies to find optimal solutions to complex problems will only continue to grow in the coming years, and research in this field will be valuable to almost all aspects of life. 

This report details the implementation of a command line general-purpose mathematical optimization engine, capable of solving LP (linear programming) problems, ILP (integer linear programming) problems, UNLP (unconstrained non-linear programming) problems, and CNLP (constrained non-linear programming) problems. In designing a general-purpose optimization solver, real-world usage and time-to-implement were weighed carefully to select the best combination of supported problem types. For maximum generality, optimizations specific to a narrow use-case, such as quadratic programming or heuristic programming, were excluded. The problems are specified within a text file, and running the optimizer on the file will generate a solution file with the ".sol" extension. 

This project will primarily be described in sections covering syntax analysis, linear programming, integer linear programming, unconstrained non-linear programming, and constrained non-linear programming. The syntax analysis section will cover the logic for parsing an input file and generating an AST (abstract syntax tree / abstract expression tree). The linear programming section details the use of a dual-simplex method for solving LP problems. The integer linear programming section describes the extension of the LP solver using a branch and bound technique. The unconstrained non-linear programming section details the application of the BFGS (Broyden-Fletcher-Goldfarb-Shanno) variant of a quasi-Newton approach to solve UNLP problems. Lastly, the constrained non-linear programming section describes the extension of the UNLP solver using an Augmented Lagrangian based penalty method for solving CNLP problems.

研究期間：2025年4月~2025年8月

# 1. link 

[GitHub Repository](https://github.com/Meilan39/General-Purpose-Solver)

[Report](https://drive.google.com/file/d/1yJx2eTkkq1azpNaemE7USJTMrvd7-Xew/view?usp=sharing)

[Presentation](https://drive.google.com/file/d/1u065r2w8etG84sPSfCPvYDg5pfiHELC6/view?usp=sharing)

## 2. Syntax Analysis

### 2.1. Overview
Syntax Analysis is a general term used to describe a procedural parsing of text with a known grammar or structure. In this case, the "grammar" that dictates how input files are structured will be specified by a BNF (Backus-Naur form), which will then be translated almost directly into a recursive parsing scheme. Although there are many styles of BNFs, the ones shown in this report will be given in the form described in Table 1.

**Table 1: BNF style used within this report**

| Symbol | Description | Range |
| :--- | :--- | :--- |
| `::=` | Define the left side with the right side | |
| `<>` | Wrapped around text to represent variables | |
| `{}` | Used like a regular parenthesis | |
| `\|` | Either the left side, or the right side (OR) | |
| `*` | The previous element can be repeated however many times | `{0,}` |
| `+` | The previous element must be repeated more than 1 time | `{1,}` |
| `?` | The previous element must be repeated 0 or 1 times | `{0, 1}` |


The high-level structure for a LP problem file is given in Program 1. The variable `<lp>` is defined as an `<objective>`, followed by a `<linear_expression>`, a semicolon, the `Constrain` keyword, and any number of `<linear_constraint>`s. This can be understood more clearly when shown in reference to Program 2, where an example LP problem file is shown. The `Minimize` in Program 2 represents the `<objective>` in the BNF, `6x1 + 3x2` represents the `<linear_expression>` for the expression $6x_1+3x_2$, and so on. 

**Program 1: High-level structure for a LP problem file**
```text
<lp> ::= <objective> <linear_expression> ; 
        Constrain {<linear_constraint>}*
```

**Program 2: An example LP problem file**
```text
Minimize
    6x1 + 3x2;
Constrain
    1.4x1 + x2 >= 7;
    -2x1 + x2 <= 2;
    x2 <= 5.5;
```

Variables like the `<lp>` shown in Program 1 are defined using other lower-level variables such as `<objective>`, which are defined elsewhere in a similar manner. Ultimately, variables of a BNF are broken down into indivisible units, which will be referred to as 'words' in this report. Program 3 shows the definition of `<comparison>`, which can be any of the listed comparative operators. Each operator is an indivisible unit, or 'word', with a specific meaning that would be lost if broken down further.

**Program 3: Example of a word shown in the definition of `<comparison>`**
```text
<comparison> ::= = | <= | >= | > | <
```

The goal of syntax analysis is to form an abstract syntax tree in three steps. The first step, called lexical analysis, is where the words of an input file are extracted and stored in a list. The second step, called syntactic analysis, is where the list of words is matched to a particular structure specified in the BNF, and a syntax tree or parse tree is generated as a result. The final step, called semantic analysis, is where further meaning is derived from the parse tree, and unnecessary details are removed to produce an AST, which represents the abstracted semantics of the input text. 

Lexical analysis can be implemented relatively simply, by extracting all valid words from a BNF and assigning a unique integer to each. This way, the input file, given as an array of characters, can be converted into an array of integers representing the order of all valid words. This significantly simplifies the syntax analysis, where the integer list is recursively parsed almost exactly as specified by the BNF.

### 2.2. Implementation
Lexical analysis is explained first, starting with Program 4 which shows the part of the code to hash a string to a unique enumerated integer. For example, `+` is hashed to `lt_plus`, which is a unique enum as defined in Program 5. The `ct_terminator` and `lt_terminator` are used to ensure that all enumerated integers are unique.

**Program 4: hashing words to unique enumerated integers**
```c
const l_Map l_map[] = {
    {"+", lt_plus},
    {"-", lt_minus},
    {"tan", lt_tan},
    {NULL, 0}
};
```

**Program 5: enumerated integers**
```c
typedef enum l_Types {
    lt_plus = ct_terminator + 1,
    lt_minus,
    lt_tan,
    /* terminator */
    lt_terminator
} l_Types;
```

Program 6 shows the definition of the `l_lex` function which handles the top-level processing for lexical analysis. The input file is read line-by-line, and is processed as either a word token, a numerical constant, or a variable name. The parsing for numerical constants are specified in a different BNF with its own lexical and syntactical analysis. Additionally, variables are defined to be a case sensitive alphabet followed by an optional number.

**Program 6: l_lex function implementation**
```c
int l_lex(Token** head, FILE* fptr) {
    int type;
    double value;
    while(fgets(l_buffer, FILE_LINE_LENGTH, fptr) != NULL) {
        // replace new line with null-terminator
        l_buffer[strcspn(l_buffer, "\n")] = '\0';
        // lexical analysis
        char* s = l_buffer;
        while(*s != '\0') {
            /* pass spaces */
            while(*s == ' ') { s++; }
            if(*s == '\0') { break; }
            /* general init */
            type = 0;
            value = 0;
            /* numerical constants */
            if((type = l_hash(&s))) { goto P; }
            if((type = c_constant(&s, &value))) { goto P; }
            if((type = l_variable(&s, &value))) { goto P; }
            /* lexical error */
            return -1;
P:          /* pushback */
            t_push(head, type, value);
        }
    }
    return 0;
}
```

The `l_lex` function appends the matched tokens to a list, which is then passed to the `s_syn` function shown in Program 7. A recursive search begins with the single `s_file` function call, and the resulting parse tree is returned as a pointer to the root. A debug file named "depthmap" is used to log all function calls during the recursive search. An error is triggered by a failure to read the entire token list or a NULL pointer to the parse tree.

**Program 7: s_syn function implementation**
```c
int s_syn(Node** head, Token* token) {
    /* open depth map */
    s_syntax_depth_map = fopen("./meta/depthmap.txt", "w");
    if(!s_syntax_depth_map) { printf("unable to open depthmap\n"); return -1; }
    /* parse */
    *head = s_file(&token, 0);
    /* close depth map */
    fclose(s_syntax_depth_map);
    /* print abstract syntax tree */
    if((*head) == NULL) return -1;
    if(token != NULL) return -1;
    return 0;
}
```

The `s_file` shown in Program 9 is the first function call in the recursive search, and is modeled off of the `File` term of the BNF, shown in Program 8. Program 8 shows that a `File` is defined to be either a `<ilp>`, `<lp>`, `<cnlp>`, or `<unlp>`. This is modeled in Program 9 with goto statements, which partition each OR condition. Within each OR condition, a function corresponding to each of the 4 types is called, which each consist of recursive calls of their own. Ultimately, the resulting parse sub-trees are appended to the current tree and returned.

**Program 8: BNF for "File"**
```text
<File> ::= <ilp> | <lp> | <cnlp> | <unlp>
```


**Program 9: s_file function implementation**
```c
Node* s_file(Token** token, int depth) {
    PRINTMAP(depth, "file", token)
    Node* node = n_construct(nt_file, 0);
    Token* ptoken = *token;
    if(!n_push(node, s_ilp(token, depth+1))) goto c2;
    goto t;
c2: *token = ptoken; n_reset(node);
    if(!n_push(node, s_lp(token, depth+1))) goto c3;
    goto t;
c3: *token = ptoken; n_reset(node);
    if(!n_push(node, s_cnlp(token, depth+1))) goto c4;
    goto t;
c4: *token = ptoken; n_reset(node);
    if(!n_push(node, s_unlp(token, depth+1))) goto f;
    goto t;
f:  *token = ptoken;
    return n_free(node);
t:  return node;
}
```

## 3. Linear Programming

### 3.1. Overview
Simply stated, linear programming is the problem of optimizing a linear objective function subject to a set of linear constraints. While systems of linear equalities can be solved directly using linear algebra, linear programming requires a different approach because the feasible region defined by inequalities is typically a convex polyhedron, and the solution can lie anywhere along its boundary. The simplex method is an iterative algorithm that can optimize LP problems by operating on a simplex "tableau" representation of the problem. This tableau resembles and is operated on as if it were a matrix [1]. 

A two-phase simplex method is considered in this particular implementation, making it capable of handling inequality constraints in addition to equality constraints. Given a maximization problem in the following form:

$$
Maximize: \qquad\qquad\qquad\qquad\qquad\qquad\\
    Z = 6x_1 + 3x_2 \\
Constraints: \qquad\qquad\qquad\qquad\qquad\qquad\\
    1.4x_1 + x_2 \le 7 \\
    -2x_1 + x_2 \le 2 \\
    x_2 \ge 1
$$

Where the variables are assumed to all be positive, i.e. $x_1, x_2 \ge 0$. The inequality conditions are turned into equality conditions by adding slack variables like so:

$$
Maximize: \qquad\qquad\qquad\qquad\qquad\qquad\\
    Z = 6x_1 + 3x_2 \\
Constraints: \qquad\qquad\qquad\qquad\qquad\qquad\\
    1.4x_1 + x_2 + s_1 = 7 \\
    -2x_1 + x_2 + s_2 = 2 \\
    x_2 - s_3 = 1 \\
$$

Where $s_1, s_2, s_3 \ge 0$ are slack variables. Since slack variables are added for $\le$ conditions and subtracted for $\ge$ conditions, they take up the "slack" to satisfy the equality. Additionally, artificial variables $y_i$ are added where slack variables are negative, and a new minimization problem for $y_1$ is tacked onto the current problem.

$$
Minimize: \qquad\qquad\qquad\qquad\qquad\qquad\\
    y_1 \\
Maximize: \qquad\qquad\qquad\qquad\qquad\qquad\\
    Z = 6x_1 + 3x_2 \\
Constraints: \qquad\qquad\qquad\qquad\qquad\qquad\\
    1.4x_1 + x_2 + s_1 = 7 \\
    -2x_1 + x_2 + s_2 = 2 \\
    x_2 - s_3 + y_1 = 1 \\
$$

The newly added minimization problem is called the Phase I operation, and the Phase II operation is the original maximization problem. The problem is then translated into the following table format resembling an augmented coefficient matrix, but with two additional rows labeled "Phase I" and "Phase II". The Phase II row is populated by simply negating the coefficients of the objective function. The Phase I row is defined to be the sum of all of the rows with an artificial variable.

| | $X_1$ | $X_2$ | $S_1$ | $S_2$ | $S_3$ | $Y_1$ | RHS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Constraints** | +1.400 | +1.000 | +1.000 | +0.000 | +0.000 | +0.000 | +7.000 |
| | -2.000 | +1.000 | +0.000 | +1.000 | +0.000 | +0.000 | +2.000 |
| | +0.000 | +1.000 | +0.000 | +0.000 | -1.000 | +1.000 | +1.000 |
| **Phase II** | -6.000 | -3.000 | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 |
| **Phase I** | +0.000 | +1.000 | +0.000 | +0.000 | -1.000 | +0.000 | +1.000 |


For Phase I, the pivot column is determined by the smallest positive value in the Phase I row. The pivot row is then determined by the smallest non-negative ratio of the RHS to the corresponding value in the pivot column. A matrix pivot operation is then conducted on the selected pivot. This process is repeated until all of the values in the Phase I row are less than or equal to 0. After one pivot operation, the tableau is given as follows, and Phase I is finished.

| | $X_1$ | $X_2$ | $S_1$ | $S_2$ | $S_3$ | $Y_1$ | RHS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Constraints** | +1.400 | +0.000 | +1.000 | +0.000 | +1.000 | -1.000 | +6.000 |
| | -2.000 | +0.000 | +0.000 | +1.000 | +1.000 | -1.000 | +1.000 |
| | +0.000 | +1.000 | +0.000 | +0.000 | -1.000 | +1.000 | +1.000 |
| **Phase II** | -6.000 | +0.000 | +0.000 | +0.000 | -3.000 | +3.000 | +3.000 |
| **Phase I** | +0.000 | +0.000 | +0.000 | +0.000 | +0.000 | -1.000 | +0.000 |


Before moving onto Phase II, the Phase I row and artificial variable columns can be deleted or ignored. 

| | $X_1$ | $X_2$ | $S_1$ | $S_2$ | $S_3$ | RHS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Constraints** | +1.400 | +0.000 | +1.000 | +0.000 | +1.000 | +6.000 |
| | -2.000 | +0.000 | +0.000 | +1.000 | +1.000 | +1.000 |
| | +0.000 | +1.000 | +0.000 | +0.000 | -1.000 | +1.000 |
| **Phase II** | -6.000 | +0.000 | +0.000 | +0.000 | -3.000 | +3.000 |


Phase II is much like Phase I, but because it is a maximization operation, the iteration continues as long as all values in the Phase II row are not greater than or equal to 0, and the column selection for the pivot chooses the most negative column rather than the most positive. After one iteration, the tableau meets the Phase II condition and looks as follows.

| | $X_1$ | $X_2$ | $S_1$ | $S_2$ | $S_3$ | RHS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Constraints** | +1.000 | +0.000 | +0.714 | +0.000 | +0.714 | +4.286 |
| | +0.000 | +0.000 | +1.429 | +1.000 | +2.429 | +9.571 |
| | +0.000 | +1.000 | +0.000 | +0.000 | -1.000 | +1.000 |
| **Phase II** | +0.000 | +0.000 | +4.286 | +0.000 | +1.286 | +28.714 |


Translating this tableau back into equation form gives the following:
$$x_1 = 4.286 - 0.714s_1 - 0.714s_3$$
$$s_2 = 9.517 - 1.429s_1 - 2.429s_3$$
$$x_2 = 1.000 + 1.000s_3$$
$$Z = 28.71 - 4.286s_1 - 1.286s_3$$

Setting $s_1 = s_3 = 0$ yields the solution to the LP problem:
$x_1 = 4.286$
$x_2 = 1$
$s_2 = 9.517$
$Z = 28.71$

Minimization problems are solved by converting them to their dual. For instance:
$$Minimize: Z = 6x_1 + 3x_2$$
would become the following, where all terms are negated [1]:
$$Maximize: -Z = -6x_1 - 3x_2$$

### 3.2. Implementation
Program 14 shows the implementation of the `simplex` function, which is called by the Main function for LP problems. In the first two lines, an object of the `Tableau` class is instantiated using the AST and solved using the `solve` method. The solution vector is then pulled from the tableau and printed to a ".sol" file with the same name as the input file.

**Program 14: simplex function implementation**
```cpp
void simplex::simplex(Node* head, Variables variables, const std::string& path) {
    Tableau tableau(head, variables);
    if(tableau.solve() == -1) {
        if(tableau.flag == sf_unbounded) printf("Error: unbounded constraints\n");
        if(tableau.flag == sf_infeasible) printf("Error: infeasible constraints\n");
        if(tableau.flag == sf_nonstandard) printf("Error: non-standard input\n");
        return;
    }
    std::vector<double> solution = tableau.solution();
    /* print solution */
    std::string sol = path;
    sol.replace(sol.rfind('.'), sol.back(), ".sol");
    FILE* fptr = fopen(sol.c_str(), "w");
}
```

Program 15 shows the template for the `Tableau` class. The `Tableau` class has a single 2-dimensional vector of doubles to represent the table, and several integer members that represent its dimensions. The important methods are those declared public. Specifically, the constructor to construct a tableau from an AST, and the `solve` method to run the simplex algorithm on the instance.

**Program 15: Tableau class template**
```cpp
class Tableau {
public:
    int row, column, defined, slack, artificial;
    int flag;
    bool minimize;
    std::vector<std::vector<double>> mat;
public:
    Tableau(Node* head, Variables variables);
    int solve();
    std::vector<double> solution();
private:
    int get_pivot(std::pair<int, int> &piv, int obj_row);
    bool optimal(int obj_row);
    void pivot(const std::pair<int, int> &piv);
    void print();
};
```

The implementation of the `solve` method is shown in Program 16. The first chunk of code ensures that the RHS is non-negative, i.e. the input is given in standard form. Phase I and Phase II are solved in independent loops, each of which iteratively pick a pivot and execute the pivot operation until they are optimal. The if-statement between the two phases is a check for tableau infeasibility in the transition stage.

**Program 16: solve function implementation**
```cpp
int Tableau::solve() {
    for(int r = 0; r < this->row - 2; r++) {
        if(cmp(this->mat[r][this->column - 1], "<", 0)) {
            this->flag = sf_nonstandard;
            return -1;
        }
    }
    /* phase one */
    while(!optimal(this->row - 1)) {
        std::pair<int, int> piv;
        if(get_pivot(piv, this->row - 1) == -1) return -1;
        pivot(piv);
    }
    if(cmp(this->mat[this->row - 1][this->column - 1], "!=", 0)) {
        this->flag = sf_infeasible;
        return -1;
    }
    /* phase two */
    while(!optimal(this->row - 2)) {
        std::pair<int, int> piv;
        if(get_pivot(piv, this->row - 2) == -1) return -1;
        pivot(piv);
    }
    return 0;
}
```

Program 17 shows the `get_pivot` method, which implements the pivot search described above. 

**Program 17: get_pivot function implementation**
```cpp
int Tableau::get_pivot(std::pair<int, int> &piv, int obj_row) {
    bool minimize = obj_row == this->row - 1;
    double extrema, temp;
    /* column */
    extrema = this->mat[obj_row][0];
    piv.second = 0;
    int end = this->column - (minimize ? 1 : this->artificial + 1);
    for(int c = 1; c < end; c++) { // excluding b
        temp = this->mat[obj_row][c];
        if((minimize && cmp(temp, ">", extrema)) ||
           (!minimize && cmp(temp, "<", extrema))) {
            extrema = temp;
            piv.second = c;
        }
    }
    /* row */
    bool empty = true;
    for(int r = 0; r < this->row - 2; r++) { // excluding phase 1 and 2
        if(cmp(this->mat[r][this->column - 1], "<", 0) ||
           cmp(this->mat[r][piv.second], "<=", 0)) continue; // non-negative
        temp = this->mat[r][this->column - 1] / this->mat[r][piv.second];
        if(empty || cmp(temp, "<", extrema)) {
            extrema = temp;
            piv.first = r;
            empty = false;
        }
    }
    if(empty) {
        if(minimize) this->flag = sf_infeasible;
        else this->flag = sf_unbounded;
        return -1;
    }
    return 0;
}
```

Program 18 shows the implementation of the `Tableau` constructor. The first portion of code scans through the AST to determine the dimensions of the matrix. In the second portion of code, each element of the AST is read again, populating the Phase II row when the objective function is read, and populating the appropriate constraint row if a constraint is read. Care is taken to ensure that duplicate variables, or variables on the wrong side of the inequality are also correctly accounted for in the tableau.

**Program 18: Tableau constructor implementation**
```cpp
Tableau::Tableau(Node* head, Variables* variables) {
    Node* mode = head->next[0];
    if(mode->next[0]->type == lt_minimize) this->minimize = true;
    if(mode->next[0]->type == lt_maximize) this->minimize = false;
    /* determine matrix size */
    this->defined = variables->len;
    this->row = 0;
    this->slack = 0;
    this->artificial = 0;
    for(int i = 0; i < mode->length; i++) {
        if(mode->next[i]->type != nt_linear_constraint) continue;
        if(mode->next[i]->next[1]->type == lt_less || 
           mode->next[i]->next[1]->type == lt_leq) {
            this->slack++;
        }
        if(mode->next[i]->next[1]->type == lt_equal) {
            this->artificial++;
        }
        if(mode->next[i]->next[1]->type == lt_greater || 
           mode->next[i]->next[1]->type == lt_geq) {
            this->slack++;
            this->artificial++;
        }
        this->row++;
    }
    this->row += 2; // ... phase II + phase I
    this->column = this->defined + this->slack + this->artificial + 1; // ... b
    /* size matrix */
    this->mat = std::vector<std::vector<double>>
                (this->row, std::vector<double>(this->column, 0.0));
    /* define objective and constraint row */
    int phase_two = this->row - 2; // objective function row
    int phase_one = this->row - 1;
    int con_row = 0;
    int countSlack = 0, countArtificial = 0;
    bool negative;
    /* populate matrix */
    for(int i = 0; i < mode->length; i++) {
        Node* element = mode->next[i];
        /* populate objective function row */
        if(element->type == nt_linear_additive) {
            Node* additive = element;
            /* for each linear multiplication */
            for(int j = 0; j < additive->length; j++) {
                Node* multiplicative = additive->next[j];
                Node *coefficient = nullptr, *variable = nullptr;
                negative = multiplicative->subtype == 2;
                for(int k = 0; k < multiplicative->length; k++) {
                    if(multiplicative->next[k]->type == nt_real)
                        coefficient = multiplicative->next[k];
                    if(multiplicative->next[k]->type == lt_variable)
                        variable = multiplicative->next[k];
                }

                /* populate matrix */
                if(variable) {
                    double value = coefficient ? n_get_value(coefficient) : 1;
                    if(negative ^ !this->minimize) value = -value;
                    this->mat[phase_two][(int)n_get_value(variable)] += value;
                } else {
                    double value = n_get_value(coefficient);
                    if(negative ^ !this->minimize) value = -value;
                    this->mat[phase_two][this->column - 1] += value;
                }
            }
        }
        /* populate constraint rows */
        if(element->type == nt_linear_constraint) {
            bool negate = false; 
            bool less, isSlack, isArtificial;
            for(int l = 0; l < element->length; l++) {
                Node* additive = element->next[l];
                /* inequality and sign flipping */
                if(additive->type != nt_linear_additive) {
                    negate = true;
                    less = additive->type == lt_less || additive->type == lt_leq;
                    isSlack = additive->type != lt_equal;
                    isArtificial = !less;
                    continue;
                }
                /* for each linear multiplication */
                for(int j = 0; j < additive->length; j++) {
                    Node* multiplicative = additive->next[j];
                    Node *coefficient = nullptr, *variable = nullptr;
                    negative = multiplicative->subtype == 2;
                    for(int k = 0; k < multiplicative->length; k++) {
                        if(multiplicative->next[k]->type == nt_real)
                            coefficient = multiplicative->next[k];
                        if(multiplicative->next[k]->type == lt_variable)
                            variable = multiplicative->next[k];
                    }
                    /* populate matrix */
                    if(variable) {
                        double value = coefficient ? n_get_value(coefficient) : 1;
                        if(negative ^ negate) value = -value;
                        this->mat[con_row][(int)n_get_value(variable)] += value;
                    } else {
                        double value = n_get_value(coefficient);
                        if(negative ^ !negate) value = -value; // constants are !negate
                        this->mat[con_row][this->column - 1] += value;
                    }
                }
            }
            /* slack variable */
            if(isSlack) 
                this->mat[con_row][this->defined + (countSlack++)] = less ? 1 : -1;
            if(isArtificial) {
                for(int j = 0; j < this->column; j++)
                    this->mat[phase_one][j] += this->mat[con_row][j];
                this->mat[con_row][this->defined + this->slack + (countArtificial++)] = 1;
            }
            con_row++; // next constraint row
        }
    }
}
```

## 4. Integer Linear Programming

### 4.1. Overview

The motivation of Integer Linear Programming is that variables which model real-world values tend to have integer restrictions. For example, there is no point in knowing that exporting 4.231 apples and 2.124 oranges is the most profitable. Furthermore, when these values are restricted to integers, the optimal export may become 3 apples and 3 oranges, which is an arbitrary result that cannot be inferred simply by looking at the decimal optimum. 

Restricting the search to integers is relatively simple with a functioning simplex engine. Specifically, this implementation will use a simple branch and bound method, by adding constraints onto the existing AST and recursively pruning the search space. The search is started by first running the simplex method on the inputted AST. If the integer variables are found to be integers after the first iteration, the search is over. If one or more of the integer bounded variables are decimal values, the most 'non-integer', or $x_{max} = max(\lvert x_i - \lfloor x_i + 0.5\rfloor \rvert)$ where $x_i(i=1,2,...n)$ represents the solution to the i'th variable, is chosen. The chosen variable, say $x_j$, becomes the branching point for two searches: one with the added constraint $x_j \le \lfloor x_{max}\rfloor$, and another with the constraint $x_j \ge \lceil x_{max}\rceil$. This process is recursively repeated until all of the integer bounded variables become integers, or the maximum recursive depth is reached. Additionally, because the solution to the Simplex algorithm at each step represents the absolute optimum for the given constraints, branches with a function value below a known solution that meet the integer bounds may be pruned.

### 4.2. Implementation
Program 19 shows the implementation of the `bnb` function, which is the starting point for the branch and bound algorithm. The AST is parsed for the index of the "Integer" delimiter and which variables are integer bound. The "branch" calls the branch and bound, and the solution is printed to a solution file of the same name as the input file.

**Program 19: bnb function implementation**
```cpp
void bnb::bnb(Node* head, Variables variables, const char* path) {
    ...
    for(int i = 0; i < mode->length; i++) {
        if(mode->next[i]->type == lt_integer) insidx = i;
        if(mode->next[i]->type == nt_variable_constraint) {
            general.push_back(mode->next[i]->next[0]->value);
        }
    }
    bnb::branch(head, variables, general, optimal, 0);
    ...
}
```

Program 20 shows the `branch` function implementation. The first four chunks of code handle the recursive depth limit, simplex function call, pruning for branches with a function value less than the best known optimum, and finding the most 'non-integer' variable, respectively. The true portion of the if-statement handles cases where all variables are integers, in which case the optimum is updated if the solution fares more optimal. The false portion continues by inserting a constraint into the AST before branching.

**Program 20: branch function implementation**
```cpp
void bnb::branch(Node* head, Variables variables, const ivec &general, fvec &optimal, int depth) {
    if(depth > bnb::maxDepth) {
        if(!maxReached) {
            printf("Warning: branch limit exceeded results may be unoptimal\n");
            maxReached = true;
        }
        return;
    }
    fvec solution = simplex::bnb(head, variables);
    if(solution.empty()) return;
    if(!optimal.empty()) {
        if(cmp(solution.back(), bnb::minimize ? ">" : "<=", optimal.back())) return;
    }
    double diffmax = 0;
    int idx = -1;
    for(int i : general) {
        double diff = abs(solution[i] - round(solution[i]));
        if(cmp(diff, "!=", 0)) {
            if(diffmax < diff) {
                diffmax = diff;
                idx = i;
            }
        }
    }
    if(idx == -1) {
        if(optimal.empty()) {
            optimal = solution;
            return;
        }
        if((bnb::minimize && cmp(solution.back(), "<", optimal.back())) ||
           (!bnb::minimize && cmp(solution.back(), ">", optimal.back()))) {
            optimal = solution;
        }
    } else {
        Node* mode = head->next[0];
        /* left side */
        if(!bnb::exists(head, idx, floor(solution[idx]), true)) {
            Node* node = bnb::new_constraint(idx, floor(solution[idx]), true);
            n_insert(mode, node, bnb::insidx);
            bnb::branch(head, variables, general, optimal, depth + 1);
            n_delete(mode, bnb::insidx);
        }
        /* right side */
        if(!bnb::exists(head, idx, ceil(solution[idx]), false)) {
            Node* node = bnb::new_constraint(idx, ceil(solution[idx]), false);
            n_insert(mode, node, bnb::insidx);
            bnb::branch(head, variables, general, optimal, depth + 1);
            n_delete(mode, bnb::insidx);
        }
    }
}
```

## 5. Unconstrained Non-linear Programming

### 5.1. Overview
Although the particular implementation discussed in this report will force a bound constraint on the search space for each variable, UNLP problems generally refer to non-linear problems with no constraints. As previously stated, the goal of a UNLP problem is to find the local and global minima of a given non-linear function. This could theoretically be accomplished by calculating the roots of the first derivative, but the challenge lies in formulating a differentiation algorithm generic enough to handle the types of functions that require optimization in real-world applications. This goes without saying that non-differentiable functions will be automatically ruled out. 

This has been a topic of interest for the math field for centuries, and efficient iterative methods to "search" for local minima have been devised. One of the aforementioned search algorithms is the ubiquitous Newton method, which leverages the fact that a given function $f(x)$ has the following second order Taylor approximation around $x=a$:

$$f(a+t) \approx f(a) + f'(a)t + \frac{f''(a)}{2!}t^2$$

Differentiating with respect to $t$ and solving gives the root. 

$$\frac{df(a)}{dx} + \frac{d^2f(a)}{dx^2}t = 0$$
$$t = -\frac{f'(a)}{f''(a)}$$

Assuming that the second derivative of $f(x)$ at $x=a$ is positive, the $t$ derived above represents the minimum of the second-order Taylor approximation of $f(x)$, giving a relatively good step direction and length to minimize the original function. This process can be repeated iteratively, updating $a$ with $t$ at every step to incrementally encroach on a local minimum for any well-defined function. 

For example, Figure 6 shows the Newton method in 2-dimensions, where $f(x)=x^4-3x^2+2x$ is represented by the red graph, and its second-order Taylor approximation at $x=-1.2$ is represented by the black graph. The blue dots represent $a$ at every step, which are given by the minimum of the previous approximation, and ultimately approach the local minima at approximately $x=-0.43$.

**Figure 6: Newton Method in 2-dimensions**
![Figure 6: Newton Method in 2-dimensions](/assets/2025-5-28-General-Purpose-Solver/newton.svg)

The Newton method can be extended into N-dimensions by replacing the first derivative with a gradient matrix, and the second derivative with a Hessian matrix [2].

$$t = -H^{-1}(x)\Delta f(x)$$

Where the final update function is given by the following:

$$x_{n+1} = x_n - H^{-1}(x_n)\Delta f(x_n)$$

Because Hessian calculation can be costly for high dimensional problems, Quasi-Newton methods such as BFGS gradually refined an approximation of the Hessian using finite-differences of the gradient at each step. The updated equation is given as follows, where $s_k$ represents the step direction scaled by the step size, and $y_k$ is the difference of the gradient at the current iteration and the previous iteration.

$$H_{k+1} = H_k + \frac{(s_k^Ty_k + y_k^TH_ky_k)(s_ks_k^T)}{(s_k^Ty_k)^2} 
                - \frac{H_ky_ks_k^T + s_ky_k^TH_k}{s_k^Ty_k}$$

This Hessian approximation relies on $s_k^Ty_k$ being positive, which occurs when the objective function is strongly convex at $x_k$, but does not necessarily hold true otherwise. In this implementation, the strong Wolfe condition will be applied to the line-search algorithm, ensuring positive curvature at all points excluding the initial starting point. Failure to ensure positive curvature at the initial point will not cause a critical failure in terms of convergence but could potentially ill-condition the Hessian leading to an erratic first several iterations. Here, the initialization shown in equation (3) will be used to precondition the Hessian appropriately.

$$H_0 = \frac{y_k^Ts_k}{y_k^Ty_k}$$

To impose the strong Wolfe condition on the line search, a line search is conducted on the step direction vector derived by the negative product of the gradient and Hessian. Regardless of the dimensions of the problem, a line search becomes a 1-dimensional search for a scalar $\alpha$ such that the new iterate $x_{k+1} = x_k - \alpha H^{-1}(x_n)\Delta f(x_n)$ satisfies the strong Wolfe conditions shown below.

$$f(x_k + \alpha_k p_k) \le f(x_k) + c_1 \alpha_k \Delta f_k^T(x_k)p_k$$

$$|\Delta f(x_k + \alpha_k p_k)^Tp_k| \le c_2 |\Delta f_k^T(x_k)p_k|$$

The first condition, also called the sufficient decrease condition or the Armijo condition, defines regions where the decrease in the objective function is largest compared to the distance from the initial point. The second condition, also called the curvature condition, defines regions where the derivative of the function is less than the derivative at the initial point. 

The regions meeting the first condition are shown shaded in purple on the left graph of Figure 7. The regions that satisfy the second conditions are shown shaded in red on the right graph of Figure 7. Figure 7 is a line search for the function $f(x,y)=sin(x)+cos(y)$ at (3,6) with search direction (1,-0.2). Running the line-search on these conditions yields an $\alpha$ value of 0.98, which can be seen to lie within both regions of the graph. [3][4]

**Figure 7: Wolfe Condition 1 (left) and Wolfe Condition 2 (right)**
<p align="center">
  <img src="/assets/2025-5-28-General-Purpose-Solver/wolfe-1.svg" width="48%">
  <img src="/assets/2025-5-28-General-Purpose-Solver/wolfe-2.svg" width="48%">
</p>

The implementations shown in programs 21 and 22, use the algorithm featured in "Numerical Optimization"[3].

**Program 21: Line Search Algorithm [3]**
```text
Algorithm 3.5 (Line Search Algorithm).
Set α0 = 0, choose αmax > 0 and α1 ∈ (0, αmax);
i = 1;
repeat
    Evaluate φ(αi);
    if φ(αi) > φ(0) + c1αiφ'(0) or [φ(αi) ≥ φ(αi-1) and i > 1]
        α* = zoom(αi-1, αi) and stop;
    Evaluate φ'(αi);
    if |φ'(αi)| ≤ -c2φ'(0)
        set α* = αi and stop;
    if φ'(αi) ≥ 0
        set α* = zoom(αi, αi-1) and stop;
    Choose αi+1 ∈ (αi, αmax);
    i = i + 1;
end (repeat)
```

**Program 22: Zoom Algorithm [3]**
```text
Algorithm 3.6 (zoom).
repeat
    Interpolate (using quadratic, cubic, or bisection) to find a trial step length αj between αlo and αhi;
    Evaluate φ(αj);
    if φ(αj) > φ(0) + c1αjφ'(0) or φ(αj) ≥ φ(αlo)
        αhi = αj;
    else
        Evaluate φ'(αj);
        if |φ'(αj)| ≤ -c2φ'(0)
            Set α* = αj and stop;
        if φ'(αj)(αhi - αlo) ≥ 0
            αhi = αlo;
        αlo = αj;
end (repeat)
```


The combination of a BFGS gradient descent and a Wolfe condition-based line search makes for a robust algorithm for local minima convergence, but solving for the global minimum still remains a challenge. In this implementation, random sampling will be used to select many initial points for BFGS, and the local minima with the lowest function value will be selected as the global minimum. In order to generalize the random sampling process, bounds will be enforced on all variables. A Mersenne twister with a uniform distribution will be used to sample a number of points proportional to the product of the volume of the variable bound and the hard-coded density constant. These points are then clustered according to proximity and a gradient condition. Points that lie on flat areas are more aggressively clustered, noting that points with small gradients take longer to converge, and provide less insight about the function. The big idea is that areas where many points gather are considered local minima with a high degree of certainty, thus legitimizing this optimization.

### 5.2. Implementation
A matrix class is first devised to more clearly express the operations of the theory presented above. Program 23 shows the template for the matrix class, which consists of a 2-dimensional vector of doubles, and a suite of methods to perform basic matrix operations. For example, the `T` method returns a copy of the object transposed. The `at` methods are both used to access a specific element of the matrix but, one returns a constant double and the other returns a mutable reference to the actual matrix element. Additionally, operator overloading is used to allow for intuitive operations. The presumption is that relevant scalars are also expressed as objects of the Matrix class and can thus use the same operations.

**Program 23: Matrix class template**
```cpp
class Matrix {
private:
    std::vector<std::vector<double>> mat;
    bool transpose = false;
public:
    int row, col;
public:
    Matrix(int r, int c, bool identity);
    Matrix(double constant);
    Matrix();
    Matrix T() const;
    double norm() const;
    double at(int r, int c) const;
    double& at(int r, int c);
    void print() const;
    Matrix operator-(const Matrix& A);
    Matrix operator+(const Matrix& A, const Matrix& B);
    Matrix operator-(const Matrix& A, const Matrix& B);
    Matrix operator*(const Matrix& A, const Matrix& B);
    Matrix operator/(const Matrix& A, const Matrix& B);
};
```

UNLP problems are processed first by the `gd` function shown in Program 24. Much of the function has been left out for clarity, but the basic structure can still be observed. First, the local minima are calculated by the `solve` function and stored in a vector "minima". If no local minima are found, the function returns with an error. Lastly, the minima are sorted by function value in the order dictated by whether it is a maximization or minimization problem.

**Program 24: gd function implementation**
```cpp
void gd::gd(Node* head, Variables variables, const std::string &path) {
    ...
    minima = gd::solve(head, variables);
    /* no solution */
    if(minima.empty()) {
        printf("Warning: No solutions found\n");
        goto E;
    }
    /* sort minima */
    std::sort(minima.begin(), minima.end(), [](auto const &a, auto const &b) {
        if(gd::maximize) return std::get<1>(a) > std::get<1>(b);
        else return std::get<1>(a) < std::get<1>(b);
    });
    ...
}
```

Program 25 shows the implementation of the `solve` function, which mainly handles the generation of the mesh and calling of the BFGS algorithm. The first chunk of code reads the bound information from the abstract syntax tree and initializes a Bound object for each variable. The Bound object consists of two integers, min and max, which represent the upper and lower bound of the variables' range. At the same time, the bounds of the uniform distribution are initialized. In the case that all variables are not correctly bounded, the function returns with an error. In the next step, the previously calculated "sampleVolume" and constant "sampleDensity" are multiplied to define the "sampleSize", which is then passed to the "mesh" function which handles the generation and clustering of samples. The generated points are then processed one-by-one, calling `BFGS` on each. When a new local minimum is found, it is appended to the vector "minima", and when a previously found minimum is found, the corresponding counter is incremented.

**Program 25: solve function implementation**
```cpp
std::vector<gd::Minima> gd::solve(Node* head, Variables variables) {
    /* read bound information */
    double sampleVolume = 1;
    for(int i = 3; i < head->next[0]->length; i++) {
        Node* next = head->next[0]->next[i];
        if(next->type == lt_constrain) break;
        Bound bound = {n_get_value(next->next[0]) * (next->next[0]->subtype==2?-1:1),
                       n_get_value(next->next[2]) * (next->next[2]->subtype==2?-1:1)};
        sampleVolume *= abs(bound.max - bound.min);
        bounds[n_get_value(next->next[1])] = bound;
        dists[n_get_value(next->next[1])] = uniform(bound.min, bound.max);
    }
    /* check for incorrectly bounded variables */
    bool unbounded = false;
    for(size_t i = 0; i < dists.size(); i++) {
        if(dists[i] == Udist) {
            printf("Error: %s is unbounded\n", variables->arr[i]);
            unbounded = true;
        }
    } 
    if(unbounded) return minima;
    
    gd::sampleSize = sampleVolume * (gd::AL ? gd::augmentedDensity : gd::sampleDensity);
    points = gd::mesh(F, variables, dists, gd::sampleSize);
    gd::sampleSize = points.size();
    
    /* call BFGS on every point */
    for(auto &point : points) {
        double minimum;
        bool cluster = false;
        if(BFGS(F, variables, point, bounds) == -1) continue;
        for(auto &m : minima) {
            if(cmp((std::get<0>(m) - point.xk).norm(), ">", gd::overlapThreshold)) continue;
            cluster = true;
            std::get<2>(m)++;
            break;
        }
        if(cluster) continue;
        bool temp = gd::AL;
        gd::AL = false;
        gd::evaluate(F, point.xk, minimum);
        gd::AL = temp;
        minima.push_back(std::make_tuple(point.xk, minimum, 1));
    }
}
```


Program 26 shows the implementation of the BFGS function. While the norm of the gradient is greater than the gradience threshold, i.e. the function has not reached a critical point, the step direction `pk` and step size `ak` is used to update the iterate `xk`. Note that the matrices initialized at the top of the function have their dimensions specified as parameters to their constructors.

**Program 26: BFGS function implementation**
```cpp
int gd::BFGS(Node* F, Variables variables, Point &point, std::vector<Bound> &bound) {
    while(cmp(gk.norm(), ">", gd::gradTolerance)) {
        if(depth > gd::maxDepth) goto E;
        fprintf(fplot, "%lf %lf\n", xk.at(0,0), xk.at(1,0));
        /* step direction */
        pk = -Hk * gk;
        /* step size */
        if(gd::line_search(F, xk, pk, gk, ak) == -1) goto E;
        sk = ak * pk;
        /* step */
        xk = xk + sk;
        if(gd::outbound(bound, xk)) goto E;
        /* pre-update */
        if(gd::gradient(F, xk, gt) == -1) goto E;
        yk = gt - gk;
        skyk = sk.T() * yk;
        if(depth == 0) Hk = Hk * (skyk / (yk.T() * yk));
        /* update hessian */
        Hk = Hk + (skyk + yk.T() * Hk * yk) * (sk * sk.T()) / (skyk * skyk)
             - (Hk * yk * sk.T() + sk * yk.T() * Hk) / skyk;
        /* update */
        gk = gt;
        depth++;
    }
}
```

Program 27 shows the gradient function, which uses the `evaluate` function to calculate a centered finite-difference. Here, `h` is defined to be the cube root of the machine epsilon for double precision, which gives optimal precision for centered finite-difference methods [5].

**Program 27: gradient function implementation**
```cpp
int gd::gradient(Node* F, const Matrix &xk, Matrix &gk) {
    double temp, Fm, Fp, h2 = 2 * gd::h;
    Matrix x = xk;
    for(int r = 0; r < gk.row; r++) {
        temp = x.at(r,0);
        x.at(r,0) = temp + gd::h;
        if(gd::evaluate(F, x, Fp) == -1) goto E;
        x.at(r,0) = temp - gd::h;
        if(gd::evaluate(F, x, Fm) == -1) goto E;
        gk.at(r,0) = (Fp - Fm) / h2;
        x.at(r,0) = temp;
    }
}
```

Program 28 shows the implementation of the `resolve` function. Given a function as a sub-tree of the AST, the resolve function begins the recursive operation to evaluate the expression at some vector `replace`. The `resolve` function calls the `additive` function on a double `value` and inverts the sign in the case of maximization. In the `additive` function, also shown in Program 28, the `multiplicative` function is called on each term, and the result is summed into `a`. Similarly, `multiplication` calls `exponential` which calls `primary` which may call `function`. The expression is traversed depth first and evaluated up.

**Program 28: resolve function implementation**
```cpp
int gd::resolve(Node* head, const Matrix& replace, double &value) {
    if(replace.col != 1) goto E;
    if(gd::additive(head, replace, value) == -1) goto E;
    if(gd::maximize) value = -value;
    return 0;
E:  return -1;
}

int gd::additive(Node* head, const Matrix &replace, double &value) {
    double a = 0, b;
    for(int i = 0; i < head->length; i++) {
        Node* next = head->next[i];
        if(gd::multiplicative(next, replace, b) == -1) goto E;
        a = next->subtype == 1 ? a + b : a - b;
        if(!isfinite(a)) { flag = gd_overflow; goto E; }
    }
    value = a;
    return 0;
E:  return -1;
}
```

## 6. Constrained Non-linear Programming

### 6.1. Overview
Constrained non-linear programming is the addition of non-linear constraints to non-linear programming. Because integer linear programming is already a system of linear constraints, this extension was unnecessary for LP, thus ILP was explored as an alternative. In particular, the non-linear constraints handled in this implementation are equality constraints, which are expressed as defined by the BNF in Program 39. The `cnlp` definition accepts any number of trailing `nonlinear_constraint`s, which are defined to be zero equality non-linear expressions, or just a non-linear expression. Both of these formats mean the same thing, and the constraint must be defined such that it is met when the non-linear expression is zero.

**Program 39: BNF defining non-linear constraints**
```text
<cnlp> ::= <objective> <nonlinear_expression> ; 
        Bound {<variable_bound>}* 
        Constrain {<nonlinear_constraint>}*

<nonlinear_constraint> ::= <nonlinear_expression> = 0 ;
                        | <nonlinear_expression>;
```

The overarching strategy for CNLP problems is the penalty method, which works by minimizing the objective function with an added penalty term which grows larger when the constraints are not met. By gradually increasing the coefficient of this penalty term, a minimum that does not meet the condition becomes infeasible. For example, given an objective function $f(x)=sin(x)+0.01x^3$ and a penalty $x-1=0$ and $sin(x/2)=0$, the following functions can be graphed as shown in Figure 16.

$$f_1(x) = f(x) + a(x-1)^2$$
$$f_2(x) = f(x) + a(sin(x/2))^2$$

**Figure 16: increasing the penalty term for f1 (red) and f2 (green) coefficient from left to right**
<p align="center">
  <img src="/assets/2025-5-28-General-Purpose-Solver/penalty-1.svg" width="32%">
  <img src="/assets/2025-5-28-General-Purpose-Solver/penalty-2.svg" width="32%">
  <img src="/assets/2025-5-28-General-Purpose-Solver/penalty-3.svg" width="32%">
</p>

The above shows an example of a quadratic penalty term, where a positive term is added to the objective function whenever the zero equality of the penalty does not hold. As shown in Figure 16, increasing the coefficients of these terms gradually deviates $f_1$ and $f_2$ from the original objective function. In the rightmost graph, where $a=1000$, minimums are infeasible where the zero equality does not hold, and the choice becomes where the objective function is lowest between the points meeting the constraints.

While theoretically robust, the penalty method's convergence requires the penalty coefficient to approach infinity. This is not practical for numerical applications and is often a source of ill-conditioning. The augmented Lagrangian method is introduced to address this. The augmented Lagrangian method is defined as the iterative minimization of equation (4), with the Lagrangian multiplier update given by equation (5).

$$L_A(x,\lambda;\mu) = f(x) - \sum_{i=1}^m \lambda_i c_i(x) + \frac{\mu_k}{2} \sum_{i=1}^m c_i^2(x)$$


$$\lambda_{i,k+1} = \lambda_{i,k} - \mu_k c_i(x_k)$$


Where $i \in m$ is the index of the penalty function, and $k$ is the iterator. $x_k$ is defined to be the optimum of the previous minimization.

### 6.2. Implementation
The Augmented Lagrangian method will be implemented in the same file as the gradient descent method, making several small tweaks to important functions. First, Program 40 shows the newly added `al` function, which highly resembles the `gd` function. In the `al` function, the constraints are read from the AST and added to a global vector "penalties" along with its own initialized Lagrangian multiplier. Next, a while loop is used to iteratively increase the coefficient `r` expressed as $\mu_k$ in the equations above. At each iteration, the augmented objective is minimized using BFGS, and if all of the found minima meet a penalty threshold, the loop is exited. While the threshold is not met, the Lagrange multiplier "multiplier" is updated according to equation (5) and $r$ is increased exponentially.

**Program 40: al function implementation**
```cpp
void gd::al(Node* head, Variables variables, const std::string& path) {
    ...
    for(int i = 0; i < mode->length; i++) {
        if(mode->next[i]->type == lt_constrain) {
            constraint = true;
            continue;
        }
        if(constraint) gd::penalties.push_back({mode->next[i], gd::minit});
    }
    while(depth < gd::maxAugmentDepth) {
        minima = gd::solve(head, variables);
        auto minimum = std::min_element(minima.begin(), minima.end(),
            [](auto const &a, auto const &b) {
                if(gd::maximize) return std::get<1>(a) > std::get<1>(b);
                else return std::get<1>(a) < std::get<1>(b);
            }
        );
        double temp;
        bool found = false;
        for(size_t i = 0; i < minima.size(); i++) { // delete violating optima
            bool valid = true;
            for(const auto &p : gd::penalties) {
                gd::resolve(p.function, std::get<0>(minima[i]), temp);
                if(gd::penaltyTolerance < fabs(temp)) valid = false;
            }
            if(valid) found = true;
            else minima.erase(minima.begin() + (i--));
        } 
        if(found) break;
        for(auto &p : gd::penalties) {
            if(minimum == minima.end()) break;
            p.multiplier = p.multiplier - gd::r * std::get<1>(*minimum) / 2;
        }
        gd::r *= 2;
        depth++;
    }
    ...
}
```

Program 41 shows the implementation of the `evaluate` function, which is one recursive layer above the `resolve` function shown in Program 28. When the global variable `AL` is true, the function also resolves and sums the penalties within the global "penalties" vector.

**Program 41: evaluate function implementation**
```cpp
int gd::evaluate(Node* head, const Matrix &replace, double &value) {
    double lagrange = 0, augment = 0, a;
    if(replace.col != 1) goto E;
    if(gd::additive(head, replace, value) == -1) goto E;
    if(gd::AL) {
        for(const auto &p : gd::penalties) {
            if(gd::additive(p.function, replace, a) == -1) goto E;
            lagrange += p.multiplier * a;
            augment += gd::r * (a * a);
        }
        value += lagrange + augment;
        if(!isfinite(value)) { flag = gd_overflow; goto E; }
    }
    if(gd::maximize) value = -value;
    return 0;
E:  return -1;
}
```

## 8. References

* [1] Dr. Tibor SIPOS, DECISION-MAKING METHODS IN TRANSPORTATION THE SIMPLEX METHOD (II.) TWO-PHASE SIMPLEX METHOD & DUALITY THEORY, https://ktkg.bme.hu/en/wp-content/uploads/sites/6/2020/02/02_Two-Phase-Simplex-Method-and-Duality-Theory.pdf
* [2] Neel Ghoshal, Why BFGS over Gradient Descent, https://medium.com/codechef-vit/why-bfgs-over-gradient-descent-3ecc3e7ffd
* [3] Jorge Nocedal, Stephen J. Wright, Numerical Optimization Second Edition, https://www.math.uci.edu/~qnie/Publications/NumericalOptimization.pdf
* [4] Michel Bierlaire, Bierlaire (2015) Optimization: principles and algorithms, EPFL Press. Section 11.5, https://www.youtube.com/playlist?list=PL10NOnsbP5Q7wNrYItE2GhKq05cVov97e
* [5] Timothy Sauer, NUMERICAL ANALYSIS SECOND EDITION, https://eclass.aueb.gr/modules/document/file.php/MISC249/Sauer%20-%20Numerical%20Analysis%202e.pdf