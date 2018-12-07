<?xml version="1.0" encoding="UTF-8"?>

<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:variable name="templace_validity">
  <validity>
    <not_before>2013-10-26T00:00:00</not_before>
    <not_after>2018-10-26T22:45:30</not_after>
  </validity>
</xsl:variable>

<xsl:variable name="templace_domains">
  <domains>
    <id_range>
      <min>0</min>
      <max>232</max>
    </id_range>
  </domains>
</xsl:variable>

<xsl:template match="/ros/profiles">
  <dds>
    <permissions>
      <xsl:for-each select="profile">
        <xsl:variable name="ns" select="@ns" />
        <xsl:variable name="node" select="@node" />
        <grant name="{@node}">
          <subject_name>CN=<xsl:value-of select="@node"/></subject_name>
          <xsl:copy-of select="$templace_validity" />
          <xsl:for-each select="./*[@*='DENY']">
            <deny_rule>
              <xsl:copy-of select="$templace_domains" />
              <xsl:apply-templates select="../*[@publish='DENY']" mode='publish'/>
              <xsl:apply-templates select="../*[@subscribe='DENY']" mode='subscribe'/>
              <xsl:apply-templates select="../*[@request='DENY']" mode='request'/>
              <xsl:apply-templates select="../*[@reply='DENY']" mode='reply'/>
              <xsl:apply-templates select="../*[@call='DENY']" mode='call'/>
              <xsl:apply-templates select="../*[@execute='DENY']" mode='execute'/>
            </deny_rule>
          </xsl:for-each>
          <xsl:for-each select="./*[@*='ALLOW']">
            <allow_rule>
              <xsl:copy-of select="$templace_domains" />
              <xsl:apply-templates select="../*[@publish='ALLOW']" mode='publish'/>
              <xsl:apply-templates select="../*[@subscribe='ALLOW']" mode='subscribe'/>
              <xsl:apply-templates select="../*[@request='ALLOW']" mode='request'/>
              <xsl:apply-templates select="../*[@reply='ALLOW']" mode='reply'/>
              <xsl:apply-templates select="../*[@call='ALLOW']" mode='call'/>
              <xsl:apply-templates select="../*[@execute='ALLOW']" mode='execute'/>
            </allow_rule>
          </xsl:for-each>
          <default>DENY</default>
        </grant>
      </xsl:for-each>
    </permissions>
  </dds>
</xsl:template>

<xsl:template match="topics" mode='publish'>
  <publish>
    <topics>
      <xsl:for-each select="topic">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rt<xsl:value-of select="$fqn"/></topic>
      </xsl:for-each>
    </topics>
  </publish>
</xsl:template>

<xsl:template match="topics" mode='subscribe'>
  <subscribe>
    <topics>
      <xsl:for-each select="topic">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rt<xsl:value-of select="$fqn"/></topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="services" mode='request'>
  <publish>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>Request</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>Reply</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="services" mode='reply'>
  <publish>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rr<xsl:value-of select="$fqn"/>Reply</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="service">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rq<xsl:value-of select="$fqn"/>Request</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="actions" mode='call'>
  <publish>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>raq<xsl:value-of select="$fqn"/>/cancelRequest</topic>
        <topic>raq<xsl:value-of select="$fqn"/>/get_resultRequest</topic>
        <topic>raq<xsl:value-of select="$fqn"/>/send_goalRequest</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rar<xsl:value-of select="$fqn"/>/cancelReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/get_resultReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/send_goalReply</topic>
        <topic>rat<xsl:value-of select="$fqn"/>/feedback</topic>
        <topic>rat<xsl:value-of select="$fqn"/>/status</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="actions" mode='execute'>
  <publish>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rar<xsl:value-of select="$fqn"/>/cancelReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/get_resultReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/send_goalReply</topic>
        <topic>rat<xsl:value-of select="$fqn"/>/feedback</topic>
        <topic>rat<xsl:value-of select="$fqn"/>/status</topic>
      </xsl:for-each>
    </topics>
  </publish>
  <subscribe>
    <topics>
      <xsl:for-each select="action">
        <xsl:variable name="fqn">
          <xsl:apply-templates select="."/>
        </xsl:variable>
        <topic>rar<xsl:value-of select="$fqn"/>/cancelReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/get_resultReply</topic>
        <topic>rar<xsl:value-of select="$fqn"/>/send_goalReply</topic>
      </xsl:for-each>
    </topics>
  </subscribe>
</xsl:template>

<xsl:template match="topic|service|action">
  <xsl:variable name="ns" select="../../@ns" />
  <xsl:variable name="name" select="." />
  <xsl:choose>
    <xsl:when test="substring($name, 1, 1) = '/'">
      <xsl:value-of select="$name"/>
    </xsl:when>
    <xsl:otherwise>
      <xsl:choose>
        <xsl:when test="substring($ns, string-length($ns), 1) = '/'">
          <xsl:value-of select="concat($ns,$name)"/>
        </xsl:when>
        <xsl:otherwise>
          <xsl:value-of select="concat($ns,'/',$name)"/>
        </xsl:otherwise>
       </xsl:choose>
    </xsl:otherwise>
   </xsl:choose>
</xsl:template>

</xsl:stylesheet>
