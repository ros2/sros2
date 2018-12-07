<?xml version="1.0" encoding="UTF-8"?>

<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="allow_rule">
  <allow_rule>
    <xsl:apply-templates select="domains"/>
    <xsl:if test="publish">
      <publish>
        <topics>
          <xsl:for-each select="publish/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()"/>
          </xsl:for-each>
        </topics>
      </publish>
    </xsl:if>
    <xsl:if test="subscribe">
      <subscribe>
        <topics>
          <xsl:for-each select="subscribe/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()"/>
          </xsl:for-each>
        </topics>
      </subscribe>
    </xsl:if>
  </allow_rule>
</xsl:template>

<xsl:template match="deny_rule">
  <deny_rule>
    <xsl:apply-templates select="domains"/>
    <xsl:if test="publish">
      <publish>
        <topics>
          <xsl:for-each select="publish/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()"/>
          </xsl:for-each>
        </topics>
      </publish>
    </xsl:if>
    <xsl:if test="subscribe">
      <subscribe>
        <topics>
          <xsl:for-each select="subscribe/topics">
            <xsl:sort select="topic"/>
            <xsl:apply-templates select="@*|node()"/>
          </xsl:for-each>
        </topics>
      </subscribe>
    </xsl:if>
  </deny_rule>
</xsl:template>

<xsl:template match="@*|node()">
  <xsl:copy>
    <xsl:apply-templates select="@*|node()"/>
  </xsl:copy>
</xsl:template>

</xsl:stylesheet>
