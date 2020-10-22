// Copyright (c) 2020, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POLARIS__GRAMMAR__GRAMMAR_HPP_
#define POLARIS__GRAMMAR__GRAMMAR_HPP_

#include <string>

namespace polaris
{
const char grammar[] =
  R"(
    PROGRAM                <-  STATEMENTS

    STATEMENTS             <-  (STATEMENT ';')*
    STATEMENT              <-  ASSIGNMENT / RETURN / EXPRESSION

    ASSIGNMENT             <-  'let' IDENTIFIER '=' EXPRESSION
    RETURN                 <-  'return' EXPRESSION

    EXPRESSION             <-  INFIX_EXPR(PREFIX_EXPR, INFIX_OPE)
    INFIX_EXPR(ATOM, OPE)  <-  ATOM (OPE ATOM)* {
                                precedence
                                  L == !=
                                  L < >
                                  L + -
                                  L * /
                              }

    IF                     <-  'if' '(' EXPRESSION ')' BLOCK ('else' BLOCK)?

    FUNCTION               <-  'fn' '(' PARAMETERS ')' BLOCK
    PARAMETERS             <-  LIST(IDENTIFIER, ',')

    BLOCK                  <-  '{' STATEMENTS '}'

    CALL                   <-  PRIMARY (ARGUMENTS / INDEX)*
    ARGUMENTS              <-  '(' LIST(EXPRESSION, ',') ')'
    INDEX                  <-   '[' EXPRESSION ']'

    PREFIX_EXPR            <-  PREFIX_OPE* CALL
    PRIMARY                <-  IF / FUNCTION / ARRAY / DOUBLE / INTEGER / BOOLEAN / NULL / IDENTIFIER / STRING / '(' EXPRESSION ')'

    ARRAY                  <-  '[' LIST(EXPRESSION, ',') ']'


    IDENTIFIER             <-  !KEYWORD < [a-zA-Z]+ >
    INTEGER                <- < '-'? [0-9]+ >
    DOUBLE                 <- < '-'? [0-9]+ '.' [0-9] >
    STRING                 <-  < ["] < (!["] .)* > ["] >
    BOOLEAN                <-  < 'true' / 'false' >
    NULL                   <-  'null'
    PREFIX_OPE             <-  < [-!] >
    INFIX_OPE              <-  < [-+/*<>] / '==' / '!=' >

    KEYWORD                <-  'null' | 'true' | 'false' | 'let' | 'return' | 'if' | 'else' | 'fn'

    LIST(ITEM, DELM)       <-  (ITEM (~DELM ITEM)*)?

    LINE_COMMENT           <-  '//' (!LINE_END .)* &LINE_END
    LINE_END               <-  '\r\n' / '\r' / '\n' / !.
    %whitespace            <-  ([ \t\r\n]+ / LINE_COMMENT)*
)";
}  // namespace polaris

#endif  // POLARIS__GRAMMAR__GRAMMAR_HPP_
